// TODO:
//  [ ] Setup events.
//
// Ancestral Program:
//  fn-0 00000000:
//    Nop
//    Nop
//

#include <string>
#include <functional>

#include "base/Ptr.h"
#include "base/vector.h"
#include "tools/Random.h"
#include "tools/random_utils.h"
#include "tools/BitSet.h"
#include "tools/Math.h"
#include "evo3/World.h"
#include "config/ArgManager.h"
#include "hardware/EventDrivenGP.h"
#include "hardware/AvidaGP.h"

#include "PABBConfig.h"

using hardware_t = emp::EventDrivenGP;
using state_t = emp::EventDrivenGP::State;
using affinity_t = emp::BitSet<8>;
using memory_t = typename emp::EventDrivenGP::memory_t;
using program_t = emp::EventDrivenGP::Program;
using function_t = emp::EventDrivenGP::Function;
using inst_t = typename::emp::EventDrivenGP::inst_t;
using inst_lib_t = typename::emp::EventDrivenGP::inst_lib_t;
using event_t = typename::emp::EventDrivenGP::event_t;
using event_lib_t = typename::emp::EventDrivenGP::event_lib_t;

class EventDrivenOrg : public emp::EventDrivenGP {
public:
  const program_t & GetGenome() { return GetProgram(); }
};

class PABB_Ancestral {
public:
  using org_t = EventDrivenOrg;
  using world_t = emp::World<org_t>;

  struct Loc {
    size_t x;
    size_t y;
    Loc(size_t _x = 0, size_t _y = 0) : x(_x), y(_y) { ; }
  };

protected:
  // Constant variables:
  static constexpr size_t TRAIT_ID__X_LOC = 0;
  static constexpr size_t TRAIT_ID__Y_LOC = 1;
  static constexpr size_t TRAIT_ID__DIR = 2;
  static constexpr size_t TRAIT_ID__RES = 3;
  static constexpr size_t TRAIT_ID__LAST_EXPORT = 4;
  static constexpr size_t TRAIT_ID__COPY_CNT = 5;
  static constexpr size_t TRAIT_ID__MSG_DIR = 6;

  static constexpr size_t NUM_NEIGHBORS = 4;
  static constexpr size_t NUM_ENV_STATES = 3;

  // Configurable variables:
  int RAND_SEED;
  size_t GRID_WIDTH;
  size_t GRID_HEIGHT;
  size_t GRID_SIZE;
  size_t UPDATES;

  MajorTransConfig config;
  emp::Ptr<emp::Random> random;
  emp::vector<affinity_t> affinity_table;   // A convenient affinity lookup table (int->bitset).

  emp::vector<affinity_t> env_state_affs;
  emp::vector<size_t> env_states;

  emp::Ptr<inst_lib_t> inst_lib;
  emp::Ptr<event_lib_t> event_lib;

  emp::Ptr<world_t> world;

  emp::vector<size_t> schedule;
  emp::vector<size_t> schedule_queue;

public:
  PABB_Ancestral(int argc, char* argv[], const std::string & _config_fname)
    : RAND_SEED(0), GRID_WIDTH(0), GRID_HEIGHT(0), GRID_SIZE(0), UPDATES(0),
      config(), random(), affinity_table(256), env_state_affs(), env_states(),
      inst_lib(), event_lib(), world(), schedule(), schedule_queue() {

    // Read configs.
    config.Read(_config_fname);
    auto args = emp::cl::ArgManager(argc, argv);
    if (args.ProcessConfigOptions(config, std::cout, _config_fname, "PABBConfig.h") == false) exit(0);
    if (args.TestUnknown() == false) exit(0);

    std::cout << "==============================" << std::endl;
    std::cout << "|    How am I configured?    |" << std::endl;
    std::cout << "==============================" << std::endl;
    config.Write(std::cout);
    std::cout << "==============================" << std::endl;

    // Localize experiment parameters.
    RAND_SEED = config.RANDOM_SEED();
    GRID_WIDTH = config.GRID_WIDTH();
    GRID_HEIGHT = config.GRID_HEIGHT();
    GRID_SIZE = GRID_WIDTH * GRID_HEIGHT;
    UPDATES = config.UPDATES();

    // Create random number generator.
    random = emp::NewPtr<emp::Random>(RAND_SEED);

    // Fill out the convenient affinity table.
    for (size_t i = 0; i < affinity_table.size(); ++i) {
      affinity_table[i].SetByte(0, (uint8_t)i);
    }

    // Setup environment state affinities.
    env_state_affs = {affinity_table[0], affinity_table[15], affinity_table[255]};
    env_states.resize(GRID_SIZE);

    // Setup instruction set.
    inst_lib = emp::NewPtr<inst_lib_t>();
    // Standard instructions:
    inst_lib->AddInst("Inc", hardware_t::Inst_Inc, 1, "Increment value in local memory Arg1");
    inst_lib->AddInst("Dec", hardware_t::Inst_Dec, 1, "Decrement value in local memory Arg1");
    inst_lib->AddInst("Not", hardware_t::Inst_Not, 1, "Logically toggle value in local memory Arg1");
    inst_lib->AddInst("Add", hardware_t::Inst_Add, 3, "Local memory: Arg3 = Arg1 + Arg2");
    inst_lib->AddInst("Sub", hardware_t::Inst_Sub, 3, "Local memory: Arg3 = Arg1 - Arg2");
    inst_lib->AddInst("Mult", hardware_t::Inst_Mult, 3, "Local memory: Arg3 = Arg1 * Arg2");
    inst_lib->AddInst("Div", hardware_t::Inst_Div, 3, "Local memory: Arg3 = Arg1 / Arg2");
    inst_lib->AddInst("Mod", hardware_t::Inst_Mod, 3, "Local memory: Arg3 = Arg1 % Arg2");
    inst_lib->AddInst("TestEqu", hardware_t::Inst_TestEqu, 3, "Local memory: Arg3 = (Arg1 == Arg2)");
    inst_lib->AddInst("TestNEqu", hardware_t::Inst_TestNEqu, 3, "Local memory: Arg3 = (Arg1 != Arg2)");
    inst_lib->AddInst("TestLess", hardware_t::Inst_TestLess, 3, "Local memory: Arg3 = (Arg1 < Arg2)");
    inst_lib->AddInst("If", hardware_t::Inst_If, 1, "Local memory: If Arg1 != 0, proceed; else, skip block.", emp::ScopeType::BASIC, 0, {"block_def"});
    inst_lib->AddInst("While", hardware_t::Inst_While, 1, "Local memory: If Arg1 != 0, loop; else, skip block.", emp::ScopeType::BASIC, 0, {"block_def"});
    inst_lib->AddInst("Countdown", hardware_t::Inst_Countdown, 1, "Local memory: Countdown Arg1 to zero.", emp::ScopeType::BASIC, 0, {"block_def"});
    inst_lib->AddInst("Close", hardware_t::Inst_Close, 0, "Close current block if there is a block to close.", emp::ScopeType::BASIC, 0, {"block_close"});
    inst_lib->AddInst("Break", hardware_t::Inst_Break, 0, "Break out of current block.");
    inst_lib->AddInst("Call", hardware_t::Inst_Call, 0, "Call function that best matches call affinity.", emp::ScopeType::BASIC, 0, {"affinity"});
    inst_lib->AddInst("Return", hardware_t::Inst_Return, 0, "Return from current function if possible.");
    inst_lib->AddInst("SetMem", hardware_t::Inst_SetMem, 2, "Local memory: Arg1 = numerical value of Arg2");
    inst_lib->AddInst("CopyMem", hardware_t::Inst_CopyMem, 2, "Local memory: Arg1 = Arg2");
    inst_lib->AddInst("SwapMem", hardware_t::Inst_SwapMem, 2, "Local memory: Swap values of Arg1 and Arg2.");
    inst_lib->AddInst("Input", hardware_t::Inst_Input, 2, "Input memory Arg1 => Local memory Arg2.");
    inst_lib->AddInst("Output", hardware_t::Inst_Output, 2, "Local memory Arg1 => Output memory Arg2.");
    inst_lib->AddInst("Commit", hardware_t::Inst_Commit, 2, "Local memory Arg1 => Shared memory Arg2.");
    inst_lib->AddInst("Pull", hardware_t::Inst_Pull, 2, "Shared memory Arg1 => Shared memory Arg2.");
    inst_lib->AddInst("Nop", hardware_t::Inst_Nop, 0, "No operation.");
    // Custom instructions:
    // inst_lib->AddInst("CopyInst", Inst_CopyInst, 0, "Simulates self-copying of instruction.");
    inst_lib->AddInst("Random", Inst_Random, 2, "Local memory: Arg2 => RandomUInt([0:(size_t)Arg1))");
    inst_lib->AddInst("Repro", Inst_Repro, 0, "Triggers reproduction if able.");
    inst_lib->AddInst("ReproRdy", Inst_ReproRdy, 1, "Local memory Arg1 => Ready to repro?");
    inst_lib->AddInst("Export0", Inst_Export0, 0, "Export product ID 0.");
    inst_lib->AddInst("Export1", Inst_Export1, 0, "Export product ID 1.");
    inst_lib->AddInst("Export2", Inst_Export2, 0, "Export product ID 2.");
    inst_lib->AddInst("RotCW", Inst_RotCW, 0, "Rotate orientation clockwise (90 degrees) once.");
    inst_lib->AddInst("RotCCW", Inst_RotCCW, 0, "Rotate orientation counter-clockwise (90 degrees) once.");
    inst_lib->AddInst("RotDir", Inst_RotDir, 1, "Rotate to face direction specified by Arg1 (Arg1 mod 4)");
    inst_lib->AddInst("GetDir", Inst_GetDir, 1, "Local memory Arg1 => Current direction.");
    inst_lib->AddInst("SendMsgFacing", Inst_SendMsgFacing, 0, "Send output memory as message event to faced neighbor.", emp::ScopeType::BASIC, 0, {"affinity"});
    inst_lib->AddInst("SendMsgRandom", Inst_SendMsgRandom, 0, "Send output memory as message event to random neighbor.", emp::ScopeType::BASIC, 0, {"affinity"});
    inst_lib->AddInst("SendMsg", Inst_SendMsg, 1, "Send output memory as message event to neighbor specified by local memory Arg1.", emp::ScopeType::BASIC, 0, {"affinity"});
    inst_lib->AddInst("BindEnv", Inst_BindEnv, 0, "Bind environment to appropriate function.");

    // Setup the event library.
    event_lib = emp::NewPtr<event_lib_t>(*emp::EventDrivenGP::DefaultEventLib());
    // TODO: setup events
    //  [ ] Message dispatcher
    //  [ ] BindEnv handler [ ] BindEnv dispatcher

    // Setup the world.
    world = emp::NewPtr<world_t>(random);
    world->SetGrid(GRID_WIDTH, GRID_HEIGHT, false);
    world->SetPrintFun([](org_t & hw, std::ostream & ostream){ hw.PrintState(ostream); });
    world->OnOrgPlacement([this](size_t id) { this->OnOrgPlacement(id); });

    // Setup the environment.
    for (size_t i = 0; i < env_states.size(); ++i)
      env_states[i] = random->GetUInt(0, NUM_ENV_STATES);

    // Initialize the population with single ancestor.
    // TODO: read ancestor from file
    program_t prog(inst_lib);
    prog.PushFunction(function_t(affinity_table[0]));
    for (size_t i = 0; i < 9; ++i) prog.PushInst("Nop");
    prog.PushInst("ReproRdy", 0);
    prog.PushInst("If", 0);
    prog.PushInst("SetMem", 0, 4);
    prog.PushInst("Random", 0);
    prog.PushInst("RotDir", 0);
    prog.PushInst("Repro");
    prog.PushInst("Close");

    EventDrivenOrg ancestor;
    ancestor.SetProgram(prog);

    // Inject ancestor in the middle of the world.
    size_t mid_x = GRID_WIDTH / 2;
    size_t mid_y = GRID_HEIGHT / 2;
    world->InjectAt(ancestor, GetID(mid_x, mid_y));
    schedule.emplace_back(GetID(mid_x, mid_y));
  }

  ~PABB_Ancestral() {
    world.Delete();
    inst_lib.Delete();
    event_lib.Delete();
    random.Delete();
  }

  size_t GetEnvState(size_t x, size_t y) {
    return env_states[GetID(x, y)];
  }

  size_t GetID(size_t x, size_t y) {
    return (size_t)(emp::Mod((int)x, (int)GRID_WIDTH) + emp::Mod((int)y, (int)GRID_HEIGHT) * (int)GRID_WIDTH);
  }

  Loc GetPos(size_t id) { return Loc(emp::Mod((int)id, (int)GRID_WIDTH), id / GRID_WIDTH); }

  // ============== Running the experiment. ==============
  void Run() {
    // Run Evolution.

    for (size_t ud = 0; ud < UPDATES; ++ud) {
      std::cout << "===============================" << std::endl;
      std::cout << "Update: " << ud << std::endl;

      // Randomize schedule.
      Shuffle(*random, schedule);

      // Give out CPU cycles to everyone on the schedule.
      for (size_t i = 0; i < schedule.size(); ++i) {
        size_t id = schedule[i];
        std::cout << "  Running... " << id << std::endl;
        world->GetOrg(id).PrintState();
        world->ProcessID(id, 1); // Call Process(num_inst = 1)
      }

      // Update the schedule (add positions that were previously empty).
      for (size_t i = 0; i < schedule_queue.size(); ++i) {
        schedule.emplace_back(schedule_queue[i]);
      } schedule_queue.resize(0);

      //world->Print();
    }
  }

  // ============== World signal handlers: ==============
  void OnOrgPlacement(size_t id) {
    // Configure placed organism.
    Loc pos = GetPos(id);
    org_t & org = world->GetOrg(id);
    std::cout << "Org being placed: " << id << "(" << pos.x << ", " << pos.y << ")"<< std::endl;
    org.SetTrait(TRAIT_ID__X_LOC, pos.x);
    org.SetTrait(TRAIT_ID__Y_LOC, pos.y);
    org.PrintState();
  }

  // ============== Instructions: ==============
  static void Inst_CopyInst(emp::EventDrivenGP & hw, const inst_t inst) {
    hw.SetTrait(TRAIT_ID__COPY_CNT, hw.GetTrait(TRAIT_ID__COPY_CNT) + 1);
  }

  static void Inst_Random(emp::EventDrivenGP & hw, const inst_t inst) {
    state_t & state = hw.GetCurState();
    state.SetLocal(inst.args[1], hw.GetRandom().GetUInt(0, (uint32_t)state.AccessLocal(inst.args[0])));
  }

  // @amlalejini - TODO
  static void Inst_Repro(emp::EventDrivenGP & hw, const inst_t inst) {

  }

  // @amlalejini - TODO
  static void Inst_ReproRdy(emp::EventDrivenGP & hw, const inst_t inst) {

  }

  static void Inst_Export0(emp::EventDrivenGP & hw, const inst_t inst) {
    hw.SetTrait(TRAIT_ID__LAST_EXPORT, 0);
    // @amlalejini - TODO: trigger export event.
  }

  static void Inst_Export1(emp::EventDrivenGP & hw, const inst_t inst) {
    hw.SetTrait(TRAIT_ID__LAST_EXPORT, 1);
    // @amlalejini - TODO: trigger export event.
  }

  static void Inst_Export2(emp::EventDrivenGP & hw, const inst_t inst) {
    hw.SetTrait(TRAIT_ID__LAST_EXPORT, 2);
    // @amlalejini - TODO: trigger export event.
  }

  static void Inst_RotCW(emp::EventDrivenGP & hw, const inst_t inst) {
    hw.SetTrait(TRAIT_ID__DIR, emp::Mod(hw.GetTrait(TRAIT_ID__DIR) + 1, NUM_NEIGHBORS));
  }

  static void Inst_RotCCW(emp::EventDrivenGP & hw, const inst_t inst) {
    hw.SetTrait(TRAIT_ID__DIR, emp::Mod(hw.GetTrait(TRAIT_ID__DIR) - 1, NUM_NEIGHBORS));
  }

  static void Inst_RotDir(emp::EventDrivenGP & hw, const inst_t inst) {
    state_t & state = hw.GetCurState();
    hw.SetTrait(TRAIT_ID__DIR, emp::Mod((int)state.AccessLocal(inst.args[0]), NUM_NEIGHBORS));
  }

  static void Inst_GetDir(emp::EventDrivenGP & hw, const inst_t inst) {
    state_t & state = hw.GetCurState();
    state.SetLocal(inst.args[0], hw.GetTrait(TRAIT_ID__DIR));
  }

  static void Inst_SendMsgFacing(emp::EventDrivenGP & hw, const inst_t inst) {
    state_t & state = hw.GetCurState();
    hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"facing"});
  }

  static void Inst_SendMsgRandom(emp::EventDrivenGP & hw, const inst_t inst) {
    state_t & state = hw.GetCurState();
    hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"random"});
  }

  static void Inst_SendMsg(emp::EventDrivenGP & hw, const inst_t inst) {
    state_t & state = hw.GetCurState();
    hw.SetTrait(TRAIT_ID__MSG_DIR, emp::Mod((int)state.AccessLocal(inst.args[0]), NUM_NEIGHBORS));
    hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"arg"});
  }

  static void Inst_BindEnv(emp::EventDrivenGP & hw, const inst_t inst) {
    hw.TriggerEvent("BindEnv");
  }

};




int main(int argc, char* argv[]) {
  PABB_Ancestral experiment(argc, argv, "ancestral__local_env.cfg");
  experiment.Run();
  return 0;
}

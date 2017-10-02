// TODO:
//  [ ] Save time by giving each hardware_t world_id trait.



#include <string>
#include <functional>
#include <utility>
#include <deque>
#include <fstream>
#include <sys/stat.h>

#include "base/Ptr.h"
#include "base/vector.h"
#include "config/ArgManager.h"
#include "tools/Random.h"
#include "tools/random_utils.h"
#include "tools/BitSet.h"
#include "tools/Math.h"
#include "tools/string_utils.h"
#include "hardware/EventDrivenGP.h"
#include "Evo/World.h"

#include "PABBConfig.h"

using hardware_t = emp::EventDrivenGP;
using state_t = emp::EventDrivenGP::State;
using affinity_t = typename emp::EventDrivenGP::affinity_t;
using memory_t = typename emp::EventDrivenGP::memory_t;
using program_t = emp::EventDrivenGP::Program;
using function_t = emp::EventDrivenGP::Function;
using inst_t = typename::emp::EventDrivenGP::inst_t;
using inst_lib_t = typename::emp::EventDrivenGP::inst_lib_t;
using event_t = typename::emp::EventDrivenGP::event_t;
using event_lib_t = typename::emp::EventDrivenGP::event_lib_t;

/// Wrapper around EventDrivenGP to satisfy World.h
class EventDrivenOrg : public emp::EventDrivenGP {
public:
  EventDrivenOrg(emp::Ptr<const inst_lib_t> _ilib, emp::Ptr<const event_lib_t> _elib, emp::Ptr<emp::Random> rnd=nullptr)
    : emp::EventDrivenGP(_ilib, _elib, rnd) { }
  const program_t & GetGenome() { return GetProgram(); }
};

/// Class used to run plasticity as a building block for developmental coordination/division of labor
/// ancestral environment experiments.
class PABB_Ancestral {
public:
  using org_t = EventDrivenOrg;
  using world_t = emp::World<org_t>;

  struct Loc {
    size_t x;
    size_t y;
    Loc(size_t _x = 0, size_t _y = 0) : x(_x), y(_y) { ; }
  };

  struct Birth {
    size_t src_id;
    size_t dest_id;
    Birth(size_t _src_id, size_t _dest_id) : src_id(_src_id), dest_id(_dest_id) { ; }
  };

protected:
  // Constant variables:
  static constexpr size_t TRAIT_ID__X_LOC = 0;        ///< Agent's Y location.
  static constexpr size_t TRAIT_ID__Y_LOC = 1;        ///< Agent's X location.
  static constexpr size_t TRAIT_ID__DIR = 2;          ///< Used to indicate which direction agent is facing.
  static constexpr size_t TRAIT_ID__RES = 3;          ///< Used to store how many resources this agent has collected.
  static constexpr size_t TRAIT_ID__LAST_EXPORT = 4;  ///< Used to determine most recent export (-1 if nothing exported).
  static constexpr size_t TRAIT_ID__MSG_DIR = 5;      ///< Used to determine direction of message dispatch.
  static constexpr size_t TRAIT_ID__RES_MOD = 6;      ///< Used to determine how many resources should be gained from an export.
  static constexpr size_t TRAIT_ID__EXPORTED = 7;     ///< Used to determine if program has exported on a single advance. (used to limit number of times program exports per advance)
  static constexpr size_t TRAIT_ID__REPRODUCED = 8;   ///< Used to determine if program has reproduced on this update.

  static constexpr size_t NUM_NEIGHBORS = 4;
  static constexpr size_t NUM_ENV_STATES = 3;

  static constexpr size_t DIR_UP = 0;
  static constexpr size_t DIR_LEFT = 1;
  static constexpr size_t DIR_DOWN = 2;
  static constexpr size_t DIR_RIGHT = 3;

  // == Configurable variables: ==
  // General settings.
  int RAND_SEED;
  size_t GRID_WIDTH;
  size_t GRID_HEIGHT;
  size_t GRID_SIZE;
  size_t UPDATES;
  std::string ANCESTOR_FPATH;

  // Resources & reproduction.
  double COST_OF_REPRO;
  double FAILED_REPRO_PENALTY;
  double RES_PER_UPDATE;
  double MAX_MOD;
  double MIN_MOD;
  double EXPORT_REWARD;

  // EventDrivenGP hardware configs.
  size_t HW_MAX_CORES;
  size_t HW_MAX_CALL_DEPTH;
  double HW_MIN_BIND_THRESH;

  // EventDrivenGP program configs.
  size_t PROG_MAX_FUNC_CNT;
  size_t PROG_MAX_FUNC_LEN;
  size_t PROG_MAX_ARG_VAL;

  // Mutation rates.
  double PER_BIT__AFFINITY_FLIP_RATE;
  double PER_INST__SUB_RATE;
  double PER_FUNC__SLIP_RATE;
  double PER_FUNC__FUNC_DUP_RATE;
  double PER_FUNC__FUNC_DEL_RATE;

  // Output info.
  size_t SYSTEMATICS_INTERVAL;
  size_t POP_SNAPSHOT_INTERVAL;
  std::string DATA_DIR;

  MajorTransConfig config;
  emp::Ptr<emp::Random> random;
  emp::vector<affinity_t> affinity_table;   // A convenient affinity lookup table (int->bitset).

  emp::vector<affinity_t> env_state_affs;
  emp::vector<size_t> env_states;

  emp::Ptr<inst_lib_t> inst_lib;
  emp::Ptr<event_lib_t> event_lib;

  emp::Ptr<world_t> world;

  emp::vector<size_t> schedule;
  emp::vector<char> scheduled;

  std::deque<Birth> birth_queue;

public:
  PABB_Ancestral(int argc, char* argv[], const std::string & _config_fname)
    : RAND_SEED(0), GRID_WIDTH(0), GRID_HEIGHT(0), GRID_SIZE(0), UPDATES(0),
      ANCESTOR_FPATH(),
      config(), random(), affinity_table(256), env_state_affs(), env_states(),
      inst_lib(), event_lib(), world(), schedule(), scheduled() {

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
    ANCESTOR_FPATH = config.ANCESTOR_FILE();
    MAX_MOD = config.MAX_MOD();
    MIN_MOD = config.MIN_MOD();
    RES_PER_UPDATE = config.RESOURCES_PER_UPDATE();
    EXPORT_REWARD = config.EXPORT_REWARD();
    COST_OF_REPRO = config.COST_OF_REPRO();
    FAILED_REPRO_PENALTY = config.FAILED_REPRO_PENALTY();
    HW_MAX_CORES = config.HW_MAX_CORES();
    HW_MAX_CALL_DEPTH = config.HW_MAX_CALL_DEPTH();
    HW_MIN_BIND_THRESH = config.HW_MIN_BIND_THRESH();
    PROG_MAX_FUNC_CNT = config.PROG_MAX_FUNC_CNT();
    PROG_MAX_FUNC_LEN = config.PROG_MAX_FUNC_LEN();
    PROG_MAX_ARG_VAL = config.PROG_MAX_ARG_VAL();
    PER_BIT__AFFINITY_FLIP_RATE = config.PER_BIT__AFFINITY_FLIP_RATE();
    PER_INST__SUB_RATE = config.PER_INST__SUB_RATE();
    PER_FUNC__SLIP_RATE = config.PER_FUNC__SLIP_RATE();
    PER_FUNC__FUNC_DUP_RATE = config.PER_FUNC__FUNC_DUP_RATE();
    PER_FUNC__FUNC_DEL_RATE = config.PER_FUNC__FUNC_DEL_RATE();
    SYSTEMATICS_INTERVAL = config.SYSTEMATICS_INTERVAL();
    POP_SNAPSHOT_INTERVAL = config.POP_SNAPSHOT_INTERVAL();
    DATA_DIR = config.DATA_DIRECTORY();

    // Setup output directory.
    mkdir(DATA_DIR.c_str(), ACCESSPERMS);
    if (DATA_DIR.back() != '/') DATA_DIR += '/';

    // Create random number generator.
    random = emp::NewPtr<emp::Random>(RAND_SEED);

    // Fill out the convenient affinity table.
    for (size_t i = 0; i < affinity_table.size(); ++i) {
      affinity_table[i].SetByte(0, (uint8_t)i);
    }

    // Setup environment state affinities.
    env_state_affs = {affinity_table[0], affinity_table[15], affinity_table[255]};
    env_states.resize(GRID_SIZE);

    // Setup schedule management
    scheduled.resize(GRID_SIZE, 0);

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
    inst_lib->AddInst("RandomDir", Inst_RandomDir, 1, "Local memory: Arg1 => RandomUInt([0:4)");
    inst_lib->AddInst("Repro", [this](hardware_t & hw, const inst_t & inst) { this->Inst_Repro(hw, inst); }, 0, "Triggers reproduction if able.");
    inst_lib->AddInst("ReproRdy", [this](hardware_t & hw, const inst_t & inst) { this->Inst_ReproRdy(hw, inst); }, 1, "Local memory Arg1 => Ready to repro?");
    inst_lib->AddInst("Export0", [this](hardware_t & hw, const inst_t & inst) { this->Inst_Export0(hw, inst); }, 0, "Export product ID 0.");
    inst_lib->AddInst("Export1", [this](hardware_t & hw, const inst_t & inst) { this->Inst_Export1(hw, inst); }, 0, "Export product ID 1.");
    inst_lib->AddInst("Export2", [this](hardware_t & hw, const inst_t & inst) { this->Inst_Export2(hw, inst); }, 0, "Export product ID 2.");
    inst_lib->AddInst("RotCW", Inst_RotCW, 0, "Rotate orientation clockwise (90 degrees) once.");
    inst_lib->AddInst("RotCCW", Inst_RotCCW, 0, "Rotate orientation counter-clockwise (90 degrees) once.");
    inst_lib->AddInst("RotDir", Inst_RotDir, 1, "Rotate to face direction specified by Arg1 (Arg1 mod 4)");
    inst_lib->AddInst("GetDir", Inst_GetDir, 1, "Local memory Arg1 => Current direction.");
    inst_lib->AddInst("SendMsgFacing", Inst_SendMsgFacing, 0, "Send output memory as message event to faced neighbor.", emp::ScopeType::BASIC, 0, {"affinity"});
    inst_lib->AddInst("SendMsgRandom", Inst_SendMsgRandom, 0, "Send output memory as message event to random neighbor.", emp::ScopeType::BASIC, 0, {"affinity"});
    inst_lib->AddInst("SendMsg", Inst_SendMsg, 1, "Send output memory as message event to neighbor specified by local memory Arg1.", emp::ScopeType::BASIC, 0, {"affinity"});
    inst_lib->AddInst("BindEnv", [this](hardware_t & hw, const inst_t & inst) { this->Inst_BindEnv(hw, inst); }, 0, "Bind environment to appropriate function.");

    // Setup the event library.
    event_lib = emp::NewPtr<event_lib_t>(*emp::EventDrivenGP::DefaultEventLib());
    event_lib->RegisterDispatchFun("Message", [this](hardware_t & hw, const event_t & event){ this->DispatchMessage(hw, event); });

    // Setup the world.
    world = emp::NewPtr<world_t>(random);
    world->SetGrid(GRID_WIDTH, GRID_HEIGHT, false);
    world->SetPrintFun([](org_t & hw, std::ostream & ostream) { hw.PrintState(ostream); });
    world->SetMutFun([this](org_t & hw, emp::Random & rnd) { return this->Mutate(hw, rnd); });
    world->OnOrgPlacement([this](size_t id) { this->OnOrgPlacement(id); });
    world->OnOffspringReady([this](org_t & hw) { this->OnOffspringReady(hw); });
    world->OnUpdate([this](size_t update) { this->OnUpdate(update); });

    auto & sys_file = world->SetupSystematicsFile(DATA_DIR + "systematics.csv");
    sys_file.SetTimingRepeat(SYSTEMATICS_INTERVAL);
    // Setup the environment (randomize).
    for (size_t i = 0; i < env_states.size(); ++i)
      env_states[i] = (size_t)random->GetUInt(0, NUM_ENV_STATES);

    // Initialize the population with single ancestor.
    std::ifstream ancestor_fstream(ANCESTOR_FPATH);
    if (!ancestor_fstream.is_open()) {
      std::cout << "Failed to open ancestor program file. Exiting..." << std::endl;
      exit(-1);
    }
    EventDrivenOrg ancestor(inst_lib, event_lib, random);
    ancestor.Load(ancestor_fstream);
    ancestor.SetMinBindThresh(HW_MIN_BIND_THRESH);
    ancestor.SetMaxCores(HW_MAX_CORES);
    ancestor.SetMaxCallDepth(HW_MAX_CALL_DEPTH);

    // Inject ancestor in the middle of the world.
    size_t mid_x = GRID_WIDTH / 2;
    size_t mid_y = GRID_HEIGHT / 2;
    size_t ancestor_id = GetID(mid_x, mid_y);
    world->InjectAt(ancestor, ancestor_id);
    Schedule(ancestor_id);
  }

  ~PABB_Ancestral() {
    world.Delete();
    inst_lib.Delete();
    event_lib.Delete();
    random.Delete();
  }

  // ============== Utilities: ===============
  size_t GetEnvState(size_t x, size_t y) {
    return env_states[GetID(x, y)];
  }

  size_t GetID(size_t x, size_t y) {
    return (size_t)(emp::Mod((int)x, (int)GRID_WIDTH) + emp::Mod((int)y, (int)GRID_HEIGHT) * (int)GRID_WIDTH);
  }

  /// Get cell faced by id pointing in direction dir.
  size_t GetFacing(size_t id, size_t dir) {
    // Dir:
    //  * 0: up (x, y+1); 1: left (x-1, y); 2: down (x, y-1), 3: right (x+1, y)
    dir = (size_t)emp::Mod((int)dir, (int)NUM_NEIGHBORS);
    Loc pos = GetPos(id);
    Loc facing = GetFacing(pos.x, pos.y, dir);
    return GetID(facing.x, facing.y);
  }

  /// Get cell faced by (x,y) in direction dir.
  Loc GetFacing(size_t x, size_t y, size_t dir) {
    int face_x = (int)x;
    int face_y = (int)y;
    switch(dir) {
      case DIR_UP:    ++face_y; break;
      case DIR_LEFT:  --face_x; break;
      case DIR_DOWN:  --face_y; break;
      case DIR_RIGHT: ++face_x; break;
    }
    if (face_y >= GRID_HEIGHT || face_y < 0) face_y = emp::Mod(face_y, (int)GRID_HEIGHT);
    if (face_x >= GRID_WIDTH || face_x < 0) face_x = emp::Mod(face_x, (int)GRID_WIDTH);
    return Loc((size_t)face_x, (size_t)face_y);
  }

  /// Get cell faced by pos in direction dir.
  Loc GetFacing(Loc pos, size_t dir) { return GetFacing(pos.x, pos.y, dir); }

  /// Get position in world grid given id.
  Loc GetPos(size_t id) { return Loc((size_t)emp::Mod((int)id, (int)GRID_WIDTH), id / GRID_WIDTH); }

  void DoExport(size_t id, size_t val) {
    hardware_t & hw = world->GetOrg(id);
    // Has organism already exported this update?
    if (hw.GetTrait(TRAIT_ID__EXPORTED)) return;
    hw.SetTrait(TRAIT_ID__LAST_EXPORT, val);
    double mod = hw.GetTrait(TRAIT_ID__RES_MOD);
    if (val == env_states[id]) {
      // Reward & increase modifier.
      hw.SetTrait(TRAIT_ID__RES, (mod * EXPORT_REWARD) + hw.GetTrait(TRAIT_ID__RES));
      mod = emp::Min(MAX_MOD, mod * 2.0);
    } else {
      // decrease modifier.
      mod = emp::Max(MIN_MOD, mod / 2.0);
    }
    // Update resource modifier.
    hw.SetTrait(TRAIT_ID__RES_MOD, mod);
    // Change environment.
    env_states[id] = random->GetUInt(NUM_ENV_STATES);
  }

  void DoReproduction(size_t src_id, size_t dest_id) {
    org_t & src_org = world->GetOrg(src_id);
    // Has source already reproduced?
    if (src_org.GetTrait(TRAIT_ID__REPRODUCED)) return;
    src_org.SetTrait(TRAIT_ID__REPRODUCED, 1);
    // Schedule reproduction.
    birth_queue.emplace_back(src_id, dest_id);
  }

  void ResetOrg(size_t id) {
    Loc pos = GetPos(id);
    org_t & org = world->GetOrg(id);
    org.ResetHardware();                    // Reset organism hardware.
    org.SpawnCore(0, memory_t(), true);     // Spin up main core.
    org.SetTrait(TRAIT_ID__X_LOC, pos.x);   // Configure traits.
    org.SetTrait(TRAIT_ID__Y_LOC, pos.y);
    org.SetTrait(TRAIT_ID__DIR, 0);
    org.SetTrait(TRAIT_ID__RES, 0);
    org.SetTrait(TRAIT_ID__LAST_EXPORT, -1);
    org.SetTrait(TRAIT_ID__MSG_DIR, -1);
    org.SetTrait(TRAIT_ID__RES_MOD, 1);
    org.SetTrait(TRAIT_ID__EXPORTED, 0);
    org.SetTrait(TRAIT_ID__REPRODUCED, 0);
  }

  void Schedule(size_t id) {
    if (scheduled[id]) return;
    schedule.emplace_back(id);
    scheduled[id] = 1;
  }

  /// Mutate organism function.
  /// Return number of mutation *events* that occur (e.g. function duplication, slip mutation are single events).
  size_t Mutate(org_t & hw, emp::Random & rnd) {
    program_t & program = hw.GetProgram();
    size_t mut_cnt = 0;
    // Duplicate a function?
    if (rnd.P(PER_FUNC__FUNC_DUP_RATE) && program.GetSize() < PROG_MAX_FUNC_CNT) {
      ++mut_cnt;
      const uint32_t fID = rnd.GetUInt(program.GetSize());
      program.PushFunction(program[fID]);
    }
    // Delete a function?
    if (rnd.P(PER_FUNC__FUNC_DEL_RATE) && program.GetSize() > 1) {
      ++mut_cnt;
      const uint32_t fID = rnd.GetUInt(program.GetSize());
      program[fID] = program[program.GetSize() - 1];
      program.program.resize(program.GetSize() - 1);
    }
    // For each function...
    for (size_t fID = 0; fID < program.GetSize(); ++fID) {
      // Mutate affinity
      for (size_t i = 0; i < program[fID].GetAffinity().GetSize(); ++i) {
        affinity_t & aff = program[fID].GetAffinity();
        if (rnd.P(PER_BIT__AFFINITY_FLIP_RATE)) {
          ++mut_cnt;
          aff.Set(i, !aff.Get(i));
        }
      }
      // Slip-mutation?
      if (rnd.P(PER_FUNC__SLIP_RATE)) {
        uint32_t begin = rnd.GetUInt(program[fID].GetSize());
        uint32_t end = rnd.GetUInt(program[fID].GetSize());
        if (begin < end && ((program[fID].GetSize() + (end - begin)) < PROG_MAX_FUNC_LEN)) {
          // duplicate begin:end
          ++mut_cnt;
          const size_t dup_size = end - begin;
          const size_t new_size = program[fID].GetSize() + dup_size;
          org_t::Function new_fun(program[fID].GetAffinity());
          for (size_t i = 0; i < new_size; ++i) {
            if (i < end) new_fun.PushInst(program[fID][i]);
            else new_fun.PushInst(program[fID][i - dup_size]);
          }
          program[fID] = new_fun;
        } else if (begin > end && ((program[fID].GetSize() - (begin - end)) >= 1)) {
          // delete end:begin
          ++mut_cnt;
          org_t::Function new_fun(program[fID].GetAffinity());
          for (size_t i = 0; i < end; ++i)
            new_fun.PushInst(program[fID][i]);
          for (size_t i = begin; i < program[fID].GetSize(); ++i)
            new_fun.PushInst(program[fID][i]);
          program[fID] = new_fun;
        }
      }
      // Substitution mutations?
      for (size_t i = 0; i < program[fID].GetSize(); ++i) {
        inst_t & inst = program[fID][i];
        // Mutate affinity (even if it doesn't have one).
        for (size_t k = 0; k < inst.affinity.GetSize(); ++k) {
          if (rnd.P(PER_BIT__AFFINITY_FLIP_RATE)) {
            ++mut_cnt;
            inst.affinity.Set(k, !inst.affinity.Get(k));
          }
        }
        // Mutate instruction.
        if (rnd.P(PER_INST__SUB_RATE)) {
          ++mut_cnt;
          inst.id = rnd.GetUInt(program.GetInstLib()->GetSize());
        }
        // Mutate arguments (even if they aren't relevent to instruction).
        for (size_t k = 0; k < org_t::MAX_INST_ARGS; ++k) {
          if (rnd.P(PER_INST__SUB_RATE)) {
            ++mut_cnt;
            inst.args[k] = rnd.GetInt(PROG_MAX_ARG_VAL);
          }
        }
      }
    }
    return mut_cnt;
  }

  // ============== Running the experiment. ==============
  void OnUpdate(size_t update) {
    std::cout << "Update: " << update <<  "  Pop size: " << schedule.size() << "  Ave depth: " << world->GetSystematics().GetAveDepth() << std::endl;
    // Randomize schedule.
    Shuffle(*random, schedule);
    // Give out CPU cycles to everyone on the schedule.
    // Note: Loop structure relies on overflowing size_t i. When hits -1, will be max size_t.
    for (size_t i = schedule.size() - 1; i < schedule.size(); --i) {
      size_t id = schedule[i];
      org_t & org = world->GetOrg(id);
      org.SetTrait(TRAIT_ID__EXPORTED, 0);
      org.SetTrait(TRAIT_ID__REPRODUCED, 0);
      org.IncTrait(TRAIT_ID__RES);  // Give out resources.
      world->ProcessID(id, 1); // Call Process(num_inst = 1)
    }
    // Process birth queue.
    while (!birth_queue.empty()) {
      Birth & birth = birth_queue.front(); // Who's next?
      world->DoBirthAt(world->GetOrg(birth.src_id), birth.dest_id, birth.src_id); // Do birth!
      ResetOrg(birth.src_id);
      birth_queue.pop_front();
    }
    // std::cout << "Press anything to continue..." << std::endl;
    // std::string x;
    // std::cin >> x;
  }

  void Snapshot(size_t update) {
    std::string snapshot_dir = DATA_DIR + "/pop_" + emp::to_string((int)update);
    std::string prog_filename;
    mkdir(snapshot_dir.c_str(), ACCESSPERMS);
    // For each individual in the population, dump full program description.
    for (size_t i = 0; i < world->GetSize(); ++i) {
      if (!scheduled[i]) continue;
      org_t & org = world->GetOrg(i);
      std::ofstream prog_ofstream(snapshot_dir + "/prog_" + emp::to_string((int)i) + ".gp");
      org.PrintProgramFull(prog_ofstream);
      prog_ofstream.close();
    }
  }

  void Run() {
    // Run Evolution.
    for (size_t ud = 0; ud < UPDATES; ++ud) {
      world->Update();
      if (ud % POP_SNAPSHOT_INTERVAL == 0) Snapshot(ud);
    }
    // Print everything out.
    for (size_t i = schedule.size() - 1; i < schedule.size(); --i) {
      size_t id = schedule[i];
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << "Printing... " << id << std::endl;
      std::cout << " " << "{id: " << schedule[i] << ", mc: " << world->GetOrg(schedule[i]).GetMaxCores() << "}" << std::endl;
      org_t & org = world->GetOrg(id);
      org.PrintState();
      std::cout << "          ~~~~~~~~~~~          " << std::endl;
      org.PrintProgramFull();
    }
  }

  // ============== World signal handlers: ==============
  void OnOrgPlacement(size_t id) {
    // Configure placed organism.
    ResetOrg(id);
    Schedule(id); // Add to schedule.
  }

  void OnOffspringReady(org_t & hw) {
    // Mutate offspring.
    world->DoMutationsOrg(hw);
  }
  // ============== Event Dispatchers: ==============
  /// Event: Message
  /// Description:
  ///   * Msg types that need to be dispatched:
  ///     * send - On a send message event, dispatch to neighbor given by hw.GetTrait(TRAIT_ID__MSG_DIR)
  ///     * broadcast -- On a broadcast message event, dispatch message to all neighbors.
  void DispatchMessage(hardware_t & hw, const event_t & event) {
    const size_t sender_x = (size_t)hw.GetTrait(TRAIT_ID__X_LOC);
    const size_t sender_y = (size_t)hw.GetTrait(TRAIT_ID__Y_LOC);
    if (event.HasProperty("send")) {
      const size_t dir = (size_t)hw.GetTrait(TRAIT_ID__MSG_DIR);
      // Who is the recipient?
      const Loc pos = GetFacing(sender_x, sender_y, dir);
      // Queue up the message.
      const size_t rID = GetID(pos.x, pos.y);
      if (world->IsOccupied(rID)) world->GetOrg(rID).QueueEvent(event);
    } else {
      const Loc u_pos = GetFacing(sender_x, sender_y, DIR_UP);
      const Loc d_pos = GetFacing(sender_x, sender_y, DIR_DOWN);
      const Loc r_pos = GetFacing(sender_x, sender_y, DIR_RIGHT);
      const Loc l_pos = GetFacing(sender_x, sender_y, DIR_LEFT);
      // Queue up messages.
      const size_t rID0 = GetID(u_pos.x, u_pos.y);
      const size_t rID1 = GetID(d_pos.x, d_pos.y);
      const size_t rID2 = GetID(r_pos.x, r_pos.y);
      const size_t rID3 = GetID(l_pos.x, l_pos.y);
      if (world->IsOccupied(rID0)) world->GetOrg(rID0).QueueEvent(event);
      if (world->IsOccupied(rID1)) world->GetOrg(rID1).QueueEvent(event);
      if (world->IsOccupied(rID2)) world->GetOrg(rID2).QueueEvent(event);
      if (world->IsOccupied(rID3)) world->GetOrg(rID3).QueueEvent(event);
    }
  }

  // ============== Instructions: ==============
  /// Instruction: ReproRdy
  /// Description: Local memory Arg1 => Ready to repro?
  void Inst_ReproRdy(emp::EventDrivenGP & hw, const inst_t & inst) {
    state_t & state = hw.GetCurState();
    state.SetLocal(inst.args[0], (double)(hw.GetTrait(TRAIT_ID__RES) >= COST_OF_REPRO));
  }

  /// Instruction: Repro
  /// Description: Trigger reproduction if hardware has collected sufficient resources. Otherwise
  ///              enforce penalty.
  void Inst_Repro(emp::EventDrivenGP & hw, const inst_t & inst) {
    // If organism has collected sufficient resources, trigger reproduction.
    double res = hw.GetTrait(TRAIT_ID__RES);
    if (res >= COST_OF_REPRO) {
      const size_t x = (size_t)hw.GetTrait(TRAIT_ID__X_LOC);
      const size_t y = (size_t)hw.GetTrait(TRAIT_ID__Y_LOC);
      const size_t dir = (size_t)hw.GetTrait(TRAIT_ID__DIR);
      const Loc offspring_pos = GetFacing(x, y, dir);
      hw.DecTrait(TRAIT_ID__RES, COST_OF_REPRO);
      DoReproduction(GetID(x, y), GetID(offspring_pos.x, offspring_pos.y));
    } else { // Otherwise, pay cost of failure.
      hw.SetTrait(TRAIT_ID__RES, res - FAILED_REPRO_PENALTY);
    }
  }

  /// Instruction: RandomDir
  /// Description: Local[Arg1] = RandomInt(0, NUM_DIRECTIONS)
  static void Inst_RandomDir(emp::EventDrivenGP & hw, const inst_t & inst) {
    state_t & state = hw.GetCurState();
    state.SetLocal(inst.args[0], hw.GetRandom().GetUInt(0, NUM_NEIGHBORS));
  }

  /// Instruction: Export0
  /// Description: Trigger export event, indicating that this is an 'Export0' via the event memory.
  void Inst_Export0(emp::EventDrivenGP & hw, const inst_t & inst) {
    const size_t id = GetID((size_t)hw.GetTrait(TRAIT_ID__X_LOC), (size_t)hw.GetTrait(TRAIT_ID__Y_LOC));
    DoExport(id, 0);
  }

  /// Instruction: Export1
  /// Description: Trigger export event, indicating that this is an 'Export1' via the event memory.
  void Inst_Export1(emp::EventDrivenGP & hw, const inst_t & inst) {
    const size_t id = GetID((size_t)hw.GetTrait(TRAIT_ID__X_LOC), (size_t)hw.GetTrait(TRAIT_ID__Y_LOC));
    DoExport(id, 1);
  }

  /// Instruction: Export2
  /// Description: Trigger export event, indicating that this is an 'Export2' via the event memory.
  void Inst_Export2(emp::EventDrivenGP & hw, const inst_t & inst) {
    const size_t id = GetID((size_t)hw.GetTrait(TRAIT_ID__X_LOC), (size_t)hw.GetTrait(TRAIT_ID__Y_LOC));
    DoExport(id, 2);
  }

  /// Instruction: RotCW
  /// Description: Rotate clockwise once.
  static void Inst_RotCW(emp::EventDrivenGP & hw, const inst_t & inst) {
    hw.SetTrait(TRAIT_ID__DIR, emp::Mod(hw.GetTrait(TRAIT_ID__DIR) + 1, NUM_NEIGHBORS));
  }

  /// Instruction: RotCCW
  /// Description: Rotate counter-clockwise once.
  static void Inst_RotCCW(emp::EventDrivenGP & hw, const inst_t & inst) {
    hw.SetTrait(TRAIT_ID__DIR, emp::Mod(hw.GetTrait(TRAIT_ID__DIR) - 1, NUM_NEIGHBORS));
  }

  /// Instruction: RotDir
  /// Description: Rotate to face direction specified by Local[Arg1] % NUM_NEIGHBORS.
  static void Inst_RotDir(emp::EventDrivenGP & hw, const inst_t & inst) {
    state_t & state = hw.GetCurState();
    hw.SetTrait(TRAIT_ID__DIR, emp::Mod((int)state.AccessLocal(inst.args[0]), NUM_NEIGHBORS));
  }

  /// Instruction: GetDir
  /// Description: Local[Arg1] = Current direction.
  static void Inst_GetDir(emp::EventDrivenGP & hw, const inst_t & inst) {
    state_t & state = hw.GetCurState();
    state.SetLocal(inst.args[0], hw.GetTrait(TRAIT_ID__DIR));
  }

  /// Instruction: SendMsgFacing
  /// Description: Send message to faced neighbor (as determined by hardware direction trait).
  static void Inst_SendMsgFacing(emp::EventDrivenGP & hw, const inst_t & inst) {
    state_t & state = hw.GetCurState();
    hw.SetTrait(TRAIT_ID__MSG_DIR, hw.GetTrait(TRAIT_ID__DIR));
    hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"send"});
  }

  /// Instruction: SendMsgRandom
  /// Description: Send message to random neighbor.
  static void Inst_SendMsgRandom(emp::EventDrivenGP & hw, const inst_t & inst) {
    state_t & state = hw.GetCurState();
    hw.SetTrait(TRAIT_ID__MSG_DIR, hw.GetRandom().GetUInt(0, NUM_NEIGHBORS));
    hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"send"});
  }

  /// Instruction: SendMsg
  /// Description: Send message to neighbor specified by Local[Arg1].
  static void Inst_SendMsg(emp::EventDrivenGP & hw, const inst_t & inst) {
    state_t & state = hw.GetCurState();
    hw.SetTrait(TRAIT_ID__MSG_DIR, emp::Mod((int)state.AccessLocal(inst.args[0]), NUM_NEIGHBORS));
    hw.TriggerEvent("Message", inst.affinity, state.output_mem, {"send"});
  }

  /// Instruction: BindEnv
  /// Description: Trigger BindEnv event. The function in hw's program that best matches current
  ///              environment affinity is called.
  void Inst_BindEnv(emp::EventDrivenGP & hw, const inst_t & inst) {
    const size_t id = GetID((size_t)hw.GetTrait(TRAIT_ID__X_LOC), (size_t)hw.GetTrait(TRAIT_ID__Y_LOC));
    const size_t e = env_states[id];
    hw.SpawnCore(env_state_affs[e], hw.GetMinBindThresh());
  }

};




int main(int argc, char* argv[]) {
  PABB_Ancestral experiment(argc, argv, "ancestral__local_env.cfg");
  experiment.Run();
  return 0;
}

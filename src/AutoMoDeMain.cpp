/*
 * @file <src/AutoMoDeMain.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/plugins/dynamic_loading.h>
#include <argos3/core/simulator/argos_command_line_arg_parser.h>

#include "./core/AutoMoDeFiniteStateMachine.h"
#include "./core/AutoMoDeFsmBuilder.h"
#include "./core/AutoMoDeController.h"

#include <argos3/demiurge/loop-functions/CoreLoopFunctions.h>

using namespace argos;

const std::string ExplainParameters() {
	std::string strExplanation = "Missing finite state machine configuration. The possible parameters are: \n\n"
		" -r | --readable-fsm \t Prints an URL containing a DOT representation of the finite state machine [OPTIONAL]. \n"
		" -s | --seed \t The seed for the ARGoS simulator [OPTIONAL] \n"
        " -f | --loopfunc \t The label loop function to use (muts be in .so file declared in xml) [OPTIONAL]"
		" --fsm-config CONF \t The finite state machine description [MANDATORY]\n"
		"\n The description of the finite state machine should be placed at the end of the command line, after the other parameters.";
	return strExplanation;
}

/**
 * @brief
 *
 */

CLoopFunctions* LoadCustomLoopFunction(TConfigurationNode& cConfigRoot, CSimulator& cSimulator, std::string str_loop_func) {
    CLoopFunctions* cLoopFunctions;
    try {
        std::string strLibrary;
        GetNodeAttributeOrDefault(cConfigRoot, "library", strLibrary, strLibrary);
        if(! strLibrary.empty()) {
            CDynamicLoading::LoadLibrary(strLibrary);
        }
        cLoopFunctions = CFactory<CLoopFunctions>::New(str_loop_func);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions", ex);
    }
    return cLoopFunctions;
}


int main(int n_argc, char** ppch_argv) {

	bool bHistory = false;

	bool bReadableFSM = false;
	std::vector<std::string> vecConfigFsm;
	bool bFsmControllerFound = false;
	UInt32 unSeed = 0;
    std::string strLoopFunction;

	std::vector<AutoMoDeFiniteStateMachine*> vecFsm;

	try {
		// Cutting off the FSM configuration from the command line

		int nCurrentArgument = 1;
		while(!bFsmControllerFound && nCurrentArgument < n_argc) {
			if(strcmp(ppch_argv[nCurrentArgument], "--fsm-config") == 0) {
				bFsmControllerFound = true;
				nCurrentArgument++;
				while (nCurrentArgument < n_argc) {
					vecConfigFsm.push_back(std::string(ppch_argv[nCurrentArgument]));
					nCurrentArgument++;
				}
				// Do not take the FSM configuration into account in the standard command line parsing.
				n_argc = n_argc - vecConfigFsm.size() - 1;
			}
			nCurrentArgument++;
		}
		if (!bFsmControllerFound) {
			THROW_ARGOSEXCEPTION(ExplainParameters());
		}

		// Configure the command line options
		CARGoSCommandLineArgParser cACLAP;
		cACLAP.AddFlag('r', "readable-fsm", "", bReadableFSM);

		cACLAP.AddFlag('t', "history", "", bHistory);

		cACLAP.AddArgument<UInt32>('s', "seed", "", unSeed);
        cACLAP.AddArgument<std::string>('f', "loopfunc", "", strLoopFunction);

		// Parse command line without taking the configuration of the FSM into account
		cACLAP.Parse(n_argc, ppch_argv);

		CSimulator& cSimulator = CSimulator::GetInstance();

		switch(cACLAP.GetAction()) {
    	case CARGoSCommandLineArgParser::ACTION_RUN_EXPERIMENT: {
				CDynamicLoading::LoadAllLibraries();
				cSimulator.SetExperimentFileName(cACLAP.GetExperimentConfigFile());

				// Creation of the finite state machine.

				AutoMoDeFsmBuilder cBuilder = AutoMoDeFsmBuilder();
				AutoMoDeFiniteStateMachine* pcFiniteStateMachine = cBuilder.BuildFiniteStateMachine(vecConfigFsm);

				// If the URL of the finite state machine is requested, display it.
				if (bReadableFSM) {
					std::cout << "Finite State Machine description: " << std::endl;
					std::cout << pcFiniteStateMachine->GetReadableFormat() << std::endl;
				}

				// Setting random seed. Only works with modified version of ARGoS3.
				cSimulator.SetRandomSeed(unSeed);
                cSimulator.LoadExperiment();

                if (strLoopFunction.size() > 0) {
                    LOGERR << "[INFO] Selected loopfunc: " << strLoopFunction << std::endl;
                    TConfigurationNode& cConfigRoot = cSimulator.GetConfigurationRoot();
                    CLoopFunctions* c_loop_functions = LoadCustomLoopFunction(GetNode(cConfigRoot,"loop_functions") ,cSimulator, strLoopFunction);
                    if  (c_loop_functions != NULL) {
                        cSimulator.SetLoopFunctions(*c_loop_functions);
                        c_loop_functions->Init(GetNode(cConfigRoot,"loop_functions"));
                    }
                }

				// Duplicate the finite state machine and pass it to all robots.
				CSpace::TMapPerType cEntities = cSimulator.GetSpace().GetEntitiesByType("controller");
				for (CSpace::TMapPerType::iterator it = cEntities.begin(); it != cEntities.end(); ++it) {
					CControllableEntity* pcEntity = any_cast<CControllableEntity*>(it->second);
					AutoMoDeFiniteStateMachine* pcPersonalFsm = new AutoMoDeFiniteStateMachine(pcFiniteStateMachine);
					vecFsm.push_back(pcPersonalFsm);
					try {
						AutoMoDeController& cController = dynamic_cast<AutoMoDeController&> (pcEntity->GetController());
						cController.SetFiniteStateMachine(pcPersonalFsm);
						cController.SetHistoryFlag(bHistory);
					} catch (std::exception& ex) {
						LOGERR << "Error while casting: " << ex.what() << std::endl;
					}
				}

				cSimulator.Execute();

				// Retrieval of the score of the swarm driven by the Finite State Machine
				CoreLoopFunctions& cLoopFunctions = dynamic_cast<CoreLoopFunctions&> (cSimulator.GetLoopFunctions());
				Real fObjectiveFunction = cLoopFunctions.GetObjectiveFunction();
				std::cout << "Score " << fObjectiveFunction << std::endl;

				break;
			}

    	case CARGoSCommandLineArgParser::ACTION_QUERY:
        CDynamicLoading::LoadAllLibraries();
        //QueryPlugins(cACLAP.GetQuery());
        break;
    	case CARGoSCommandLineArgParser::ACTION_SHOW_HELP:
        cACLAP.PrintUsage(LOG);
        break;
		 	case CARGoSCommandLineArgParser::ACTION_SHOW_VERSION:
        cACLAP.PrintVersion();
        break;
      case CARGoSCommandLineArgParser::ACTION_UNKNOWN:
        // Should never get here
        break;
		}

		cSimulator.Destroy();

	} catch(std::exception& ex) {
    // A fatal error occurred: dispose of data, print error and exit
    printf("Errors:\n%s\n", ex.what());
    LOGERR << ex.what() << std::endl;
#ifdef ARGOS_THREADSAFE_LOG
    LOG.Flush();
    LOGERR.Flush();
#endif
    return 1;
  }

	for (unsigned int i = 0; i < vecFsm.size(); ++i) {
		delete vecFsm.at(i);
	}

	/* Everything's ok, exit */
  return 0;
}

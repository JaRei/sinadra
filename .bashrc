export SINADRA_FOLDER=sinadra
export sinadra_repo_root=~/${SINADRA_FOLDER}

sinadra() {
	python3 ${sinadra_repo_root}/implementation/sinadra_risk_sensor_client.py
}

export scenario_runner_path=~/${SINADRA_FOLDER}/scenario_runner/scenario_runner.py
export sinadra_scenarios_path=${sinadra_repo_root}/implementation/scenarios/carla_scenarios

run_scenario() {

case "$1" in
	lf_nobrake)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing1.xosc
	;;

	lf_nobrake_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing1_ManualControl.xosc
	;;

	lf_stopline_gentle_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing2_GentleBraking_ManualControl.xosc
	;;

	lf_stopline_gentle)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing2_GentleBraking.xosc
	;;

	lf_stopline_strong_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing2_StrongBraking_ManualControl.xosc
	;;

	lf_stopline_strong)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing2_StrongBraking.xosc
	;;

	lf_twovehicles_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing3_ManualControl.xosc
	;;

	lf_twovehicles)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing3.xosc
	;;

	lf_fvlanechange)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing4_FrontLaneChange.xosc
	;;

	lf_fvlanechange_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing4_FrontLaneChange_ManualControl.xosc
	;;

	lf_overtake)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing5_Overtaken.xosc
	;;

	lf_overtake_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing5_Overtaken_ManualControl.xosc
	;;

	lf_cutin_ego)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInEgo.xosc
	;;

	lf_cutin_ego_ind)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInEgo_Indicator.xosc
	;;

	lf_cutin_ego_from_right_single)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInEgoFromRight_Single.xosc
	;;
	
	lf_cutin_ego_from_right_ind_single)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInEgoFromRight_Indicator_Single.xosc
	;;

	lf_cutin_ego_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInEgo_ManualControl.xosc
	;;

	lf_cutin_ego_ind_single)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInEgo_Indicator_Single.xosc
	;;

	lf_cutin_ego_single)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInEgo_Single.xosc
	;;

	lf_cutin_fv)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInFront.xosc
	;;

	lf_cutin_fv_ind)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInFront_Indicator.xosc
	;;


	lf_cutin_fv_man)

	python3 ${scenario_runner_path} --openscenario ${sinadra_scenarios_path}/LaneFollowing6_CutInFront_ManualControl.xosc
	;;

	


esac

}

from copy import deepcopy
from config import DEFAULT_CONFIG_FILE
from scenario_handling.Scenario import Scenario
from tools.file_handling import delete_dir, move_dir


# scenario refers to different config settings with fixed obstacles and adc routing
def create_scenarios(generated_individual, config_file_obj, pre_record_info_list, name_prefix, default):
    scenario_list = []
    for pre_record_info in pre_record_info_list:
        scenario = create_scenario(pre_record_info, name_prefix, config_file_tuned_status)
        scenario_list.append(scenario)
    return scenario_list


def create_scenario(pre_record_info, name_prefix, config_file_tuned_status):
    # generation_id = f"{name_prefix}_Scenario_{str(pre_record_info.record_id)}"
    # print(f"  Scenario Generation ID: {generation_id}")
    record_name = pre_record_info.record_name
    scenario = Scenario(record_name)
    scenario.update_record_id(pre_record_info.record_id)
    scenario.update_config_file_status(config_file_tuned_status)
    scenario.update_record_info(pre_record_info)
    return scenario

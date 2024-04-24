from copy import deepcopy
from config import DEFAULT_CONFIG_FILE, AUTOWARE_CURRENT_CONFIG_DIR_PATH, AUTOWARE_DEFAULT_CONFIG_DIR_PATH
from configurator.autoware.AutowareTranslator import AutowareTranslator
from scenario_handling.Scenario import Scenario
from tools.file_handling import delete_dir, move_dir


def config_file_generating(generated_individual, config_file_obj, default):
    new_option_obj_list = deepcopy(config_file_obj.option_obj_list)
    delete_dir(AUTOWARE_CURRENT_CONFIG_DIR_PATH, False)
    move_dir(AUTOWARE_DEFAULT_CONFIG_DIR_PATH, AUTOWARE_CURRENT_CONFIG_DIR_PATH)
    config_file_tuned_status = False  # config file not tuned
    if not default:
        # delete_dir(AUTOWARE_CURRENT_CONFIG_DIR_PATH, False)
        # move_dir(AUTOWARE_DEFAULT_CONFIG_DIR_PATH, AUTOWARE_CURRENT_CONFIG_DIR_PATH)
    # else:
        generated_value_list = generated_individual.value_list
        tuned_id_list = []
        for generated_value, option_obj in zip(generated_value_list, new_option_obj_list):
            if option_obj.option_value != generated_value:
                option_obj.option_value = generated_value
                tuned_id_list.append(option_obj.option_id)
        # if ADS_SELECT == "Apollo":
        #     output_string_list = ApolloTranslator.option_obj_translator(new_option_obj_list)
        #     ApolloTranslator.save2file(output_string_list)
        # elif ADS_SELECT == "Autoware":
        AutowareTranslator.option_obj_translator(new_option_obj_list, tuned_id_list)
        config_file_tuned_status = True  # config file tuned
    return config_file_tuned_status


# scenario refers to different config settings with fixed obstacles and adc routing
def create_scenarios(generated_individual, config_file_obj, pre_record_info_list, name_prefix, default):
    config_file_tuned_status = config_file_generating(generated_individual, config_file_obj, default)
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
    # if AV_TESTING_APPROACH != "Random":
    scenario.update_record_info(pre_record_info)
        # if TRAFFIC_LIGHT_MODE == "read":
        #     traffic_light_control = pre_record_info.traffic_lights_list
        # elif TRAFFIC_LIGHT_MODE == "random":
        #     traffic_light_control = TCSection.get_one()
        # else:
        #     traffic_light_control = None
        # scenario.update_traffic_lights(traffic_light_control)
    return scenario

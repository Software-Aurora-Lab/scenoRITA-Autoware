from pathlib import Path

PROJECT_NAME = "scenoRITA_autoware_v1"

LOGGING_PREFIX_REGEX = (
    "^(?P<severity>[DIWEF])(?P<month>\d\d)(?P<day>\d\d) "
    "(?P<hour>\d\d):(?P<minute>\d\d):(?P<second>\d\d)\.(?P<microsecond>\d\d\d) "
    "(?P<filename>[a-zA-Z<][\w._<>-]+):(?P<line>\d+)"
)
LOGGING_FORMAT = (
    "<level>{level.name[0]}{time:MMDD}</level> "
    "<green>{time:HH:mm:ss.SSS}</green> "
    "<cyan>{file}:{line}</cyan>] "
    "<bold>{message}</bold>"
)

# ENVIRONMENT SETTINGS
AV_TESTING_APPROACH = "AutowareScenarios"  # AutowareScenarios
DOCKER_CONTAINER_NAME = "scenoRITA_autoware"
MAX_RECORD_TIME = 60  # 10/30/50
AUTOWARE_CMD_PREPARE_TIME = 25
TIME_HOUR_THRESHOLD = 10  # hours  # todo:
DEFAULT_SCRIPT_PORT = 5555
CONTAINER_NUM = 3  # 3/4/5  # todo:
DOCKER_IMAGE_ID = "5d3ce2be2fc7"  # todo:

# DIRECTORIES
DIR_ROOT = str(Path(__file__).parent.parent.parent)  # outside of scenorita-autoware
PROJECT_ROOT = str(Path(__file__).parent.parent)
FEATURES_CSV_DIR = f'{PROJECT_ROOT}/data/violation_features'

ADS_ROOT = f'{DIR_ROOT}/autoware'
ADS_MAP_DIR = f'{DIR_ROOT}/autoware_maps' # todo: change to f'{DIR_ROOT}/autoware_map/autoware_scenario_data/maps'
SUPPORTED_MAPS = list(x.name for x in Path(ADS_MAP_DIR).iterdir() if x.is_dir())
ADS_SCENARIO_DIR = f'{PROJECT_ROOT}/out'
ADS_TEMP_SCENARIO_DIR = f'{PROJECT_ROOT}/out/temp_scenarios'

# TMP_RECORDS_DIR = f'{ADS_RECORDS_OUTPUT_DIR}/scenario_test_runner'
TMP_RECORDS_DIR = f'/tmp/scenario_test_runner'
ADS_RECORDS_DIR = f'{ADS_ROOT}/records/records_for_analysis'

PRE_SCRIPTS_DIR = f'{PROJECT_ROOT}/data/scripts'
MY_SCRIPTS_DIR = f"{ADS_ROOT}/scripts"

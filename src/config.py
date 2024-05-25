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
DOCKER_CONTAINER_NAME = "scenoRITA_autoware"
MAX_RECORD_TIME = 60  # max time to record a scenario
AUTOWARE_CMD_PREPARE_TIME = 25  # time to prepare for Scenario Simulator v2
DEFAULT_SCRIPT_PORT = 5555
CONTAINER_NUM = 3  # 3/4/5  # the number of docker container in parallel
DOCKER_IMAGE_ID = "5d3ce2be2fc7"  # todo: to fill in your image id

DIR_ROOT = str(Path(__file__).parent.parent.parent)
PROJECT_ROOT = str(Path(__file__).parent.parent)

ADS_ROOT = f'{DIR_ROOT}/autoware'
ADS_MAP_DIR = f'{DIR_ROOT}/autoware_maps'
SUPPORTED_MAPS = list(x.name for x in Path(ADS_MAP_DIR).iterdir() if x.is_dir())

TMP_RECORDS_DIR = f'/tmp/scenario_test_runner'

PRE_SCRIPTS_DIR = f'{PROJECT_ROOT}/data/scripts'
MY_SCRIPTS_DIR = f"{ADS_ROOT}/scripts"

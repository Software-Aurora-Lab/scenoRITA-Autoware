from pathlib import Path
from sys import modules

import xmlschema
from pkg_resources import resource_string

from autoware.scenario.validation.scenario_validation import ScenarioValidator, ReachPositionConditionValidator, \
    validate_file


def validate(paths):
    schema = xmlschema.XMLSchema(
        resource_string(__name__, "resources/OpenSCENARIO-1.2.xsd").decode("utf-8")
    )

    # scenario_validators = [ScenarioValidator, ReachPositionConditionValidator]

    # str_validators = ["ScenarioValidator", "ReachPositionConditionValidator"]

    str_validators = list()

    scenario_validators = list()
    for validator in str_validators:
        scenario_validators.append(getattr(modules[__name__], validator)())

    for path in paths.iterdir():
        if path.suffix == ".yaml":
            validate_file(path, schema, scenario_validators)

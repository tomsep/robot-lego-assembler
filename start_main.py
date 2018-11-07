from __future__ import division
import yaml
import os

from legoassembler.main import run


def _load_config():
    """ Load configuration from config.yml

    Makes file paths OS independent.

    Returns
    -------
    dict

    """

    with open('config.yml', 'r') as f:
        cfg = yaml.load(f)

    def _replace_separator(entry):
        # Make path OS independent
        return entry.replace('/', os.sep).replace('\\', os.sep)

    cfg['grip_def_script'] = _replace_separator(cfg['grip_def_script'])

    cfg['calibration_data']['camera'] = \
        _replace_separator(cfg['calibration_data']['camera'])

    cfg['calibration_data']['platform'] = \
        _replace_separator(cfg['calibration_data']['platform'])

    return cfg


if __name__ == '__main__':

    cfg = _load_config()
    run(cfg)



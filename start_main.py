from __future__ import division
import yaml
import os

from legoassembler.main import run

def _load_config():
    """ Load configuration from config.yml

    Returns
    -------
    dict

    """

    with open('config.yml', 'r') as f:
        cfg = yaml.load(f)
    _discover_scripts(cfg)
    return cfg


def _discover_scripts(config, pckg_name='legoassembler'):
    """ Construct and validate absolute paths to scripts

    Parameters
    ----------
    config : dict
        Configuration loaded from config.yml
    pckg_name : str
        Name of the package in that contains the source code.

    """

    root = os.path.dirname(os.path.abspath(__file__))
    scripts = config['ur_scripts']
    folder = pckg_name + os.sep + scripts['directory']

    for key, script in scripts.items():
        if key != 'directory':
            new_path = os.path.join(root, folder, script)
            if not os.path.isfile(new_path):
                raise ValueError('File "{}" does not exist'.format(new_path))
            scripts[key] = new_path


if __name__ == '__main__':

    cfg = _load_config()
    run(cfg)



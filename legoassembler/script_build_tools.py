from __future__ import division


def add_spaces(s, spaces):
    """ Add leading spaces for each row

    Parameters
    ----------
    s : str
    spaces : int
        Number of leading spaces to add.

    Returns
    -------
    str

    """

    return (spaces * ' ').join(('\n' + s.lstrip()).splitlines(True))

def build(script, gripper_base_def, gripper_fun_def, ip, port):
    """ Build runnable script by replacing '$$' variables with their true definitions

    $$ADDITIONS$$ are replaced with additions required to operate the gripper.
    $$GRIP(closed=100, speed=100, force=100)$$ is replaced with a function
    to actually operate the gripper.
    $$SCOKET$$ is replaced with 'socket_open(ip, port)'.

    Notes
    -----
    The script should not be inside a main level function i.e. not like
    def program()
        **code**
    end

    but like
    **code**

    Nested functions are allowed.

    Parameters
    ----------
    script : str
        Path to o a source script.
    gripper_base_def : str
        Path to a source file for gripper base definitions.
    gripper_fun_def : str
        Path to a Source file for gripper function definition.
    ip : str
        IP address to use for socket_open command.
    port : int
        Port to use for socket_open command.

    Returns
    -------
    str
        Runnable script.

    """

    # def local function
    def grip(closed, speed, force):
        """ Builds gripping function (URSCRIPT) according to given parameters

        Loads definitions from 'urscripts/gripper_function_definition.script'.

        Replaces strings $$CLOSED$$, $$SPEED$$ and $$FORCE$$ with
        given parameter values.

        Notes
        -----
        Requires gripper additions to be present in order for the generated function to
        be usable.

        Parameters
        ----------
        closed : float
            Amount closed 0..100%.
        speed : float
            Speed 0..100%
        force : float
            Force 0..100%

        Returns
        -------
        str
            Function as string for activating the gripper.

        """

        with open(gripper_fun_def, 'r') as f:
            s = f.read()

        s = s.replace('$$CLOSED$$', str(closed))
        s = s.replace('$$SPEED$$', str(speed))
        s = s.replace('$$FORCE$$', str(force))

        return s

    with open(script, 'r') as f:

        lines = f.readlines()

        for i, line in enumerate(lines):
            line = line.lower()

            if '$$grip' in line:

                # Get gripper function as string
                gripper_fun_string = eval(line.replace('$$', ''))

                # Match indention with rest of the file
                spaces = len(line) - len(line.lstrip(' '))
                gripper_fun_string = add_spaces(gripper_fun_string, spaces)

                # Replace line
                lines[i] = gripper_fun_string

            elif '$$additions' in line:
                with open(gripper_base_def, 'r') as f:
                    spaces = len(line) - len(line.lstrip(' '))
                    additions_string = f.read()
                    additions_string = add_spaces(additions_string, spaces)
                    lines[i] = additions_string

            elif '$$socket' in line:
                spaces = len(line) - len(line.lstrip(' '))
                lines[i] = add_spaces('socket_open("{}", {})'.format(ip, port), spaces)

        # Finished built script
        full_script = ''.join(lines)
        full_script = add_spaces(full_script, 4)

        full_script = 'def remote_script():\n' + full_script + '\nend\n'
        return full_script


if __name__ == '__main__':

    # Test code, writes output script to a file
    script = 'urscripts/build_simple_2.script'
    gripped_fun_def = 'urscripts/gripper_function_definition.script'
    gripper_base_def = 'urscripts/gripper_base_definitions.script'
    script = build(script, gripper_base_def, gripped_fun_def, '192.168.137.1', 30000)

    with open('generated_ur_script.script', 'w') as f:
        f.write(script)
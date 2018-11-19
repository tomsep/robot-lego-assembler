from __future__ import division, print_function


def load_file(fname):

    lines = []
    cleaned_lines = []

    # Open the LDraw file and read the lines.
    with open(fname, "r") as lego:

        print ("The LEGO house model was loaded successfully.")
        print()
        data = lego.readlines()

        # Use space to divide the row.
        for line in data:
            lines.append(line.split())

    # We are only using lines that start with value 1. Zeros are comments etc.
    for i in range(len(lines)):
        if lines[i][0] == '1':
            cleaned_lines.append(lines[i])

    # Changing the type of the block. Currently supported are 2x2 and 2x4.
    for i in range(len(cleaned_lines)):
        if cleaned_lines[i][14] == '3001.dat':
            cleaned_lines[i][14] = '2x4'

        elif cleaned_lines[i][14] == '3003.dat':
            cleaned_lines[i][14] = '2x2'

    # Changing colour code to a string. Allows user error as in some cases there are multiple options for same colour.
    for i in range(len(cleaned_lines)):

        if cleaned_lines[i][1] == '4' or cleaned_lines[i][1] == '320':
            cleaned_lines[i][1] = 'red'

        elif cleaned_lines[i][1] == '1' or cleaned_lines[i][1] == '73' or cleaned_lines[i][1] == '272':
            cleaned_lines[i][1] = 'blue'

        elif cleaned_lines[i][1] == '2' or cleaned_lines[i][1] == '10' or cleaned_lines[i][1] == '288':
            cleaned_lines[i][1] = 'green'

        elif cleaned_lines[i][1] == '14':
            cleaned_lines[i][1] = 'yellow'

        elif cleaned_lines[i][1] == '15':
            cleaned_lines[i][1] = 'white'

        elif cleaned_lines[i][1] == '0':
            cleaned_lines[i][1] = 'black'

    # Checking if the model contains unsupported colours or brick sizes. Terminating if it does.
    for i in range(len(cleaned_lines)):

        if cleaned_lines[i][1] not in ('red', 'blue', 'green', 'yellow', 'white', 'black'):
            print ("Unable to build: Model contains unsupported colours.")
            exit()

        elif cleaned_lines[i][14] not in ('2x4', '2x2'):
            print ("Unable to build: Model contains unsupported brick sizes.")
            exit()

    return cleaned_lines


def coordinates(build_order):

    # Lego Digital Designer sometimes outputs the coordinates with multiple decimals. This rounds them up.
    for i in range(len(build_order)):

        for j in range(2, 14):
            build_order[i][j] = round(float(build_order[i][j]))

    # Simplifying the coordinates and transforming them into the positive quarter.
    a_min = min(i[2] for i in build_order)
    a_min2 = min(i[4] for i in build_order)

    for i in range(len(build_order)):
        build_order[i][3] = (build_order[i][3])/(-24)
        build_order[i][2] = ((build_order[i][2])-a_min)/40
        build_order[i][4] = ((build_order[i][4])-a_min2)/40

    # Creating new list of lists for the build plan.
    # [[ layer, x_coord, y_coord, orientation, color, size ], [-||-], ... ]
    build_plan = [[] for x in range(len(build_order))]

    for i in range(len(build_order)):
        build_plan[i].append(build_order[i][3])
        build_plan[i].append(build_order[i][4])
        build_plan[i].append(build_order[i][2])

        if build_order[i][5] == 1 or build_order[i][5] == -1:
            build_plan[i].append('parallel_to_y')
        elif build_order[i][7] == 1 or build_order[i][7] == -1:
            build_plan[i].append('parallel_to_x')

        build_plan[i].append(build_order[i][1])
        build_plan[i].append(build_order[i][14])

    # Puts the bricks in right order by layer. That way first layer is actually build first and so on.
    build_plan = sorted(build_plan, key=lambda build_plan: build_plan[0])

    return build_plan


# This function goes through the build plan and calculates the number of bricks needed.
# Returns a list with both brick sizes
def number_of_bricks(plan):

    # Define variables.
    bricks = [[], []]
    count = 0
    red_2x2    = 0
    red_2x4    = 0
    blue_2x2   = 0
    blue_2x4   = 0
    green_2x2  = 0
    green_2x4  = 0
    yellow_2x2 = 0
    yellow_2x4 = 0
    white_2x2  = 0
    white_2x4  = 0
    black_2x2  = 0
    black_2x4  = 0

    for i in range(len(plan)):

        if plan[i][5] == '2x2':
            count = count + 1

            if plan[i][4] == 'green':
                green_2x2 = green_2x2 + 1
            elif plan[i][4] == 'red':
                red_2x2 = red_2x2 + 1
            elif plan[i][4] == 'blue':
                blue_2x2 = blue_2x2 + 1
            elif plan[i][4] == 'yellow':
                yellow_2x2 = yellow_2x2 + 1
            elif plan[i][4] == 'white':
                white_2x2 = white_2x2 + 1
            elif plan[i][4] == 'black':
                black_2x2 = black_2x2 + 1

    bricks[0].append(count)
    bricks[0].append(green_2x2)
    bricks[0].append(red_2x2)
    bricks[0].append(blue_2x2)
    bricks[0].append(yellow_2x2)
    bricks[0].append(white_2x2)
    bricks[0].append(black_2x2)
    count = 0

    for i in range(len(plan)):

        if plan[i][5] == '2x4':
            count = count + 1

            if plan[i][4] == 'green':
                green_2x4 = green_2x4 + 1
            elif plan[i][4] == 'red':
                red_2x4 = red_2x4 + 1
            elif plan[i][4] == 'blue':
                blue_2x4 = blue_2x4 + 1
            elif plan[i][4] == 'yellow':
                yellow_2x4 = yellow_2x4 + 1
            elif plan[i][4] == 'white':
                white_2x4 = white_2x4 + 1
            elif plan[i][4] == 'black':
                black_2x4 = black_2x4 + 1

    bricks[1].append(count)
    bricks[1].append(green_2x4)
    bricks[1].append(red_2x4)
    bricks[1].append(blue_2x4)
    bricks[1].append(yellow_2x4)
    bricks[1].append(white_2x4)
    bricks[1].append(black_2x4)

    return bricks


def printer(brick, planned):

    print("The model requires following LEGO bricks:")
    print()

    print(brick[0][0], "2x2 sized bricks (", brick[0][1], "green,", brick[0][2], "red,", brick[0][3], "blue,",
          brick[0][4], "yellow,", brick[0][5], "white and", brick[0][6], "black )")

    print(brick[1][0], "2x4 sized bricks (", brick[1][1], "green,", brick[1][2], "red,", brick[1][3], "blue,",
          brick[1][4], "yellow,", brick[1][5], "white and", brick[1][6], "black )")
    print()

    print("The first brick in the build plan is a", planned[0][4], planned[0][5], "brick.")
    print()

    print("The build plan is as follows:")
    print()

    for i in range(len(planned)):
        print(planned[i])


if __name__ == '__main__':

    fname = 'lego.ldr'
    lego_file = load_file(fname)
    plans = coordinates(lego_file)
    legos = number_of_bricks(plans)
    printer(legos, plans)


# the width and length of a DUPLO 2x2 brick is equal to the width of a LEGO 2x4 brick,
# which is 31.8mm (32mm-0.2mm). By putting DUPLO and LEGO side by side, one can also see that the DUPLO height without
# studs is twice of the size in LEGO (19.2mm). The LEGO studs below the DUPLO brick touch DUPLO walls, so DUPLO wall
# thickness is the same as LEGO wall thickness (either before spline walls were introduced to LEGO bricks or LEGO wall
# thickness plus the length of LEGO splines), which is 1.5mm. The DUPLO tube height is the height of the brick minus
# the height of a LEGO stud (which is somewhere between 1.6mm and 1.8mm). I'd better use the larger value, so the tubes
# start 1.8mm above the floor of the brick. From the top, LEGO tubes fit into DUPLO studs, so the stud inner diameter
# should equal to 6.51mm (8*sqrt(2) - 4.8$). But if you carefully look inside a DUPLO stud, this fitting is only achieved by
# four additional lugs that give a minimal inner diameter around 6mm to facilitate snapping into.
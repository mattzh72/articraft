from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


ALUMINUM = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
BLACK_RUBBER = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
DARK_PLASTIC = Material("dark_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
SAFETY_ORANGE = Material("safety_orange", rgba=(1.0, 0.45, 0.08, 1.0))
STEEL = Material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))


SLEEVE_X = (-0.165, 0.165)
SLEEVE_Z_CENTER = 0.58
SLEEVE_LENGTH = 0.86
SLEEVE_OUTER = 0.046
SLEEVE_WALL = 0.006
SLEEVE_TOP_Z = SLEEVE_Z_CENTER + SLEEVE_LENGTH / 2.0
HINGE_Y = -0.040
HINGE_Z = 0.090
AXLE_Y = 0.105
AXLE_Z = 0.095
HANDLE_TRAVEL = 0.32


def _add_box(part, name: str, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, name: str, radius: float, length: float, xyz, rpy, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_square_sleeve(frame, index: int, x_center: float) -> None:
    """Four overlapping wall boxes make one open rectangular telescoping sleeve."""
    half = SLEEVE_OUTER / 2.0
    wall = SLEEVE_WALL
    zc = SLEEVE_Z_CENTER
    y_center = 0.0

    _add_box(
        frame,
        f"sleeve_{index}_side_wall_0",
        (wall, SLEEVE_OUTER, SLEEVE_LENGTH),
        (x_center - half + wall / 2.0, y_center, zc),
        ALUMINUM,
    )
    _add_box(
        frame,
        f"sleeve_{index}_side_wall_1",
        (wall, SLEEVE_OUTER, SLEEVE_LENGTH),
        (x_center + half - wall / 2.0, y_center, zc),
        ALUMINUM,
    )
    _add_box(
        frame,
        f"sleeve_{index}_front_wall",
        (SLEEVE_OUTER, wall, SLEEVE_LENGTH),
        (x_center, y_center - half + wall / 2.0, zc),
        ALUMINUM,
    )
    _add_box(
        frame,
        f"sleeve_{index}_rear_wall",
        (SLEEVE_OUTER, wall, SLEEVE_LENGTH),
        (x_center, y_center + half - wall / 2.0, zc),
        ALUMINUM,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_aluminum_hand_truck",
        materials=[ALUMINUM, BLACK_RUBBER, DARK_PLASTIC, SAFETY_ORANGE, STEEL],
    )

    # Root frame: a narrow ladder of aluminum sleeves, cross tubes, axle, and
    # lower hinge hardware.  The two vertical sleeves are deliberately hollow so
    # the telescoping handle rails are visibly captured without colliding with
    # the sleeve walls.
    frame = model.part("frame")
    for i, x in enumerate(SLEEVE_X):
        _add_square_sleeve(frame, i, x)

    _add_box(frame, "top_crossbar", (0.355, 0.030, 0.034), (0.0, 0.036, 0.955), ALUMINUM)
    _add_box(frame, "middle_crossbar", (0.355, 0.028, 0.032), (0.0, 0.036, 0.585), ALUMINUM)
    _add_box(frame, "lower_crossbar", (0.410, 0.030, 0.034), (0.0, 0.018, 0.185), ALUMINUM)
    _add_box(frame, "toe_latch_pad", (0.120, 0.016, 0.030), (0.0, -0.012, 0.365), DARK_PLASTIC)
    _add_box(frame, "toe_latch_backplate", (0.365, 0.010, 0.034), (0.0, -0.022, 0.365), DARK_PLASTIC)

    # Rear axle and the lower side brackets that carry it back from the sleeves.
    for i, x in enumerate(SLEEVE_X):
        _add_box(frame, f"lower_upright_{i}", (0.046, 0.046, 0.160), (x, 0.0, 0.115), ALUMINUM)

    _add_cylinder(
        frame,
        "axle",
        radius=0.011,
        length=0.620,
        xyz=(0.0, AXLE_Y, AXLE_Z),
        rpy=(0.0, math.pi / 2.0, 0.0),
        material=STEEL,
    )
    for i, x in enumerate(SLEEVE_X):
        _add_box(frame, f"axle_strut_{i}", (0.040, 0.135, 0.030), (x, 0.060, AXLE_Z), ALUMINUM)
        _add_box(frame, f"base_foot_{i}", (0.052, 0.074, 0.028), (x, 0.055, 0.078), ALUMINUM)

    _add_cylinder(
        frame,
        "hinge_pin",
        radius=0.008,
        length=0.505,
        xyz=(0.0, HINGE_Y, HINGE_Z),
        rpy=(0.0, math.pi / 2.0, 0.0),
        material=STEEL,
    )
    _add_box(frame, "hinge_bracket_0", (0.026, 0.028, 0.072), (-0.238, HINGE_Y, HINGE_Z + 0.018), ALUMINUM)
    _add_box(frame, "hinge_bracket_1", (0.026, 0.028, 0.072), (0.238, HINGE_Y, HINGE_Z + 0.018), ALUMINUM)
    _add_box(frame, "hinge_cheek_0", (0.058, 0.054, 0.020), (-0.217, 0.001, HINGE_Z + 0.060), ALUMINUM)
    _add_box(frame, "hinge_cheek_1", (0.058, 0.054, 0.020), (0.217, 0.001, HINGE_Z + 0.060), ALUMINUM)

    # Telescoping handle section.  The two lower rails fit inside the hollow
    # frame sleeves; their hidden length remains inside the sleeves at full
    # extension.  The cross grip and two visible loops make the upper handle read
    # as a real pull handle rather than a single rod.
    handle = model.part("handle")
    for i, x in enumerate(SLEEVE_X):
        _add_box(handle, f"inner_rail_{i}", (0.026, 0.026, 0.780), (x, 0.0, -0.190), ALUMINUM)
        _add_box(handle, f"rail_cap_{i}", (0.037, 0.030, 0.030), (x, 0.0, 0.210), DARK_PLASTIC)
        _add_box(handle, f"guide_shoe_{i}", (0.034, 0.034, 0.032), (x, 0.0, -0.560), DARK_PLASTIC)

    _add_box(handle, "grip_bar", (0.405, 0.038, 0.045), (0.0, 0.0, 0.240), DARK_PLASTIC)
    _add_box(handle, "handle_bridge", (0.360, 0.026, 0.040), (0.0, 0.0, 0.125), ALUMINUM)
    _add_box(handle, "release_button", (0.048, 0.014, 0.026), (0.0, -0.020, 0.130), SAFETY_ORANGE)

    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )

    # Folding toe plate.  At q=0 it is folded upright against the frame. Positive
    # rotation around +X lowers it forward to a horizontal carrying shelf.
    toe_plate = model.part("toe_plate")
    _add_box(
        toe_plate,
        "plate_slab",
        (0.430, 0.026, 0.310),
        (0.0, 0.0, 0.205),
        ALUMINUM,
    )
    for i, x in enumerate((-0.145, 0.0, 0.145)):
        _add_box(toe_plate, f"raised_rib_{i}", (0.040, 0.014, 0.260), (x, -0.018, 0.220), ALUMINUM)
        _add_box(toe_plate, f"hinge_leaf_{i}", (0.038, 0.020, 0.070), (x, -0.004, 0.045), ALUMINUM)
    _add_box(toe_plate, "toe_lip", (0.430, 0.038, 0.026), (0.0, -0.010, 0.362), SAFETY_ORANGE)
    for i, (x, length) in enumerate(((-0.150, 0.115), (0.0, 0.125), (0.150, 0.115))):
        _add_cylinder(
            toe_plate,
            f"hinge_barrel_{i}",
            radius=0.017,
            length=length,
            xyz=(x, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=ALUMINUM,
        )

    model.articulation(
        "frame_to_toe_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe_plate,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=0.0, upper=math.radians(92.0)),
    )

    # Rear wheels are individual spinning links on the shared axle.
    for i, x in enumerate((-0.285, 0.285)):
        wheel = model.part(f"wheel_{i}")
        _add_cylinder(
            wheel,
            "tire",
            radius=0.090,
            length=0.055,
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=BLACK_RUBBER,
        )
        _add_cylinder(
            wheel,
            "hub",
            radius=0.036,
            length=0.075,
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=STEEL,
        )
        _add_cylinder(
            wheel,
            "side_cap",
            radius=0.030,
            length=0.010,
            xyz=(-0.038 if x < 0.0 else 0.038, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=ALUMINUM,
        )

        model.articulation(
            f"axle_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, AXLE_Y, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    toe_plate = object_model.get_part("toe_plate")
    handle_slide = object_model.get_articulation("frame_to_handle")
    toe_hinge = object_model.get_articulation("frame_to_toe_plate")

    # Captured rotating shafts are deliberately seated inside the represented
    # wheel hubs and toe-plate hinge barrels.
    for wheel_name in ("wheel_0", "wheel_1"):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="axle",
            elem_b="hub",
            reason="The shared axle is intentionally captured inside the solid wheel hub/tire proxy.",
        )
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="axle",
            elem_b="tire",
            reason="The solid tire visual represents a rubber ring around the axle bore, so the axle is intentionally nested through the wheel center.",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="yz",
            inner_elem="axle",
            outer_elem="tire",
            margin=0.0,
            name=f"{wheel_name} axle centered in wheel bore proxy",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="x",
            elem_a="axle",
            elem_b="hub",
            min_overlap=0.055,
            name=f"{wheel_name} hub remains on shared axle",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="x",
            elem_a="axle",
            elem_b="tire",
            min_overlap=0.045,
            name=f"{wheel_name} tire surrounds the axle bore proxy",
        )

    for i in range(3):
        ctx.allow_overlap(
            frame,
            toe_plate,
            elem_a="hinge_pin",
            elem_b=f"hinge_barrel_{i}",
            reason="The hinge pin is intentionally captured inside the toe-plate hinge barrels.",
        )
        ctx.expect_within(
            frame,
            toe_plate,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=f"hinge_barrel_{i}",
            margin=0.0,
            name=f"hinge pin centered in barrel {i}",
        )
        ctx.expect_overlap(
            frame,
            toe_plate,
            axes="x",
            elem_a="hinge_pin",
            elem_b=f"hinge_barrel_{i}",
            min_overlap=0.08,
            name=f"hinge barrel {i} captures the pin",
        )

    def _aabb(part, elem):
        result = ctx.part_element_world_aabb(part, elem=elem)
        if result is None:
            return None
        return tuple(tuple(v) for v in result)

    def _rail_inside_sleeve(rail_name: str, sleeve_index: int) -> bool:
        rail_box = _aabb(handle, rail_name)
        front = _aabb(frame, f"sleeve_{sleeve_index}_front_wall")
        rear = _aabb(frame, f"sleeve_{sleeve_index}_rear_wall")
        side_0 = _aabb(frame, f"sleeve_{sleeve_index}_side_wall_0")
        side_1 = _aabb(frame, f"sleeve_{sleeve_index}_side_wall_1")
        if None in (rail_box, front, rear, side_0, side_1):
            return False
        rail_min, rail_max = rail_box
        front_min, front_max = front
        rear_min, rear_max = rear
        side0_min, side0_max = side_0
        side1_min, side1_max = side_1
        return (
            rail_min[0] > side0_max[0] + 0.001
            and rail_max[0] < side1_min[0] - 0.001
            and rail_min[1] > front_max[1] + 0.001
            and rail_max[1] < rear_min[1] - 0.001
        )

    for pose_name, q in (("collapsed", 0.0), ("extended", HANDLE_TRAVEL)):
        with ctx.pose({handle_slide: q}):
            for i in range(2):
                ctx.check(
                    f"{pose_name} handle rail {i} remains inside sleeve cavity",
                    _rail_inside_sleeve(f"inner_rail_{i}", i),
                    details=f"q={q}",
                )
                ctx.expect_overlap(
                    handle,
                    frame,
                    axes="z",
                    elem_a=f"inner_rail_{i}",
                    elem_b=f"sleeve_{i}_front_wall",
                    min_overlap=0.20 if pose_name == "collapsed" else 0.18,
                    name=f"{pose_name} rail {i} retained in sleeve",
                )

    rest_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        extended_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle extends upward along the uprights",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({toe_hinge: 0.0}):
        ctx.expect_gap(
            frame,
            toe_plate,
            axis="y",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="sleeve_0_front_wall",
            negative_elem="plate_slab",
            name="folded toe plate parks close to front sleeve",
        )
        ctx.expect_overlap(
            toe_plate,
            frame,
            axes="z",
            elem_a="plate_slab",
            elem_b="sleeve_0_front_wall",
            min_overlap=0.20,
            name="folded toe plate lies against upright frame height",
        )

    with ctx.pose({toe_hinge: math.radians(92.0)}):
        toe_aabb = ctx.part_element_world_aabb(toe_plate, elem="plate_slab")
        ok = False
        if toe_aabb is not None:
            toe_min, toe_max = toe_aabb
            ok = (toe_max[1] - toe_min[1]) > 0.30 and (toe_max[2] - toe_min[2]) < 0.055
        ctx.check(
            "toe plate folds down into a horizontal shelf",
            ok,
            details=f"toe plate aabb={toe_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

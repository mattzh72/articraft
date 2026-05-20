from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _open_box_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    wall: float,
    open_direction: str,
    name: str,
):
    """Return a thin metal box/cap mesh with one open end along local Z."""
    if open_direction == "top":
        outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
        cutter = (
            cq.Workplane("XY")
            .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.002, centered=(True, True, False))
            .translate((0.0, 0.0, wall))
        )
    elif open_direction == "bottom":
        outer = (
            cq.Workplane("XY")
            .box(width, depth, height, centered=(True, True, False))
            .translate((0.0, 0.0, -height))
        )
        cutter = (
            cq.Workplane("XY")
            .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.002, centered=(True, True, False))
            .translate((0.0, 0.0, -height - 0.001))
        )
    else:
        raise ValueError(open_direction)

    shell = outer.cut(cutter)
    try:
        shell = shell.edges("|Z").fillet(0.0012)
    except Exception:
        pass
    return mesh_from_cadquery(shell, name, tolerance=0.00025, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_flip_top_lighter")

    brass = model.material("warm_brushed_brass", rgba=(0.84, 0.63, 0.28, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_steel = model.material("dark_knurled_steel", rgba=(0.16, 0.16, 0.15, 1.0))
    soot_black = model.material("dark_opening_shadow", rgba=(0.02, 0.018, 0.014, 1.0))
    wick_mat = model.material("woven_cotton_wick", rgba=(0.88, 0.78, 0.54, 1.0))

    # A pocket lighter is only a few centimeters tall.
    case_w = 0.038
    case_d = 0.013
    lower_h = 0.0385
    lid_h = 0.018
    wall = 0.00115
    hinge_r = 0.00155
    hinge_x = -case_w / 2.0 - hinge_r
    hinge_z = lower_h + 0.0005

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        _open_box_mesh(
            width=case_w,
            depth=case_d,
            height=lower_h,
            wall=wall,
            open_direction="top",
            name="lower_shell_cup",
        ),
        material=brass,
        name="lower_shell_cup",
    )
    # Two outer hinge knuckles live on the lower case half, leaving the center
    # y-segment free for the lid knuckle.
    for i, y in enumerate((-0.0048, 0.0048)):
        lower_shell.visual(
            Cylinder(radius=hinge_r, length=0.0027),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"hinge_knuckle_{i}",
        )
        lower_shell.visual(
            Box((0.0023, 0.0027, 0.0020)),
            origin=Origin(xyz=(-case_w / 2.0 - 0.00055, y, hinge_z - 0.00110)),
            material=brass,
            name=f"hinge_leaf_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        _open_box_mesh(
            width=case_w,
            depth=case_d,
            height=lid_h,
            wall=wall,
            open_direction="bottom",
            name="lid_shell_cap",
        ),
        # The lid part frame sits on the side hinge axis.  In the closed pose
        # the cap extends across +X and upward from that line.
        origin=Origin(xyz=(case_w / 2.0 + hinge_r, 0.0, lid_h)),
        material=brass,
        name="lid_shell_cap",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.0037),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.0023, 0.0037, 0.0020)),
        origin=Origin(xyz=(hinge_r - 0.00055, 0.0, 0.00075)),
        material=brass,
        name="lid_hinge_leaf",
    )

    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=0.0, upper=1.95),
    )

    insert = model.part("insert")
    # The removable insert has a slightly smaller fuel can that slips into the
    # hollow lower shell and a raised chimney/striker deck visible above it.
    insert.visual(
        Box((0.0351, 0.0092, 0.0365)),
        origin=Origin(xyz=(0.0, 0.0, 0.02025)),
        material=chrome,
        name="fuel_can",
    )
    for side, x in (("side_0", -0.01770), ("side_1", 0.01770)):
        insert.visual(
            Box((0.00030, 0.0038, 0.0180)),
            origin=Origin(xyz=(x, 0.0, 0.0220)),
            material=chrome,
            name=f"{side}_spring_rail",
        )
    insert.visual(
        Box((0.0340, 0.0102, 0.0014)),
        origin=Origin(xyz=(0.0, 0.0, lower_h + 0.00065)),
        material=chrome,
        name="top_deck",
    )
    chimney_h = 0.0145
    chimney_z = lower_h + 0.0014 + chimney_h / 2.0
    for side, y in (("front", -0.00485), ("rear", 0.00485)):
        insert.visual(
            Box((0.023, 0.00065, chimney_h)),
            origin=Origin(xyz=(0.0015, y, chimney_z)),
            material=chrome,
            name=f"{side}_chimney_wall",
        )
        # Dark round dots sit flush on the metal plates to read as punched air
        # holes without breaking the connected insert into small loose islands.
        idx = 0
        for z in (lower_h + 0.0050, lower_h + 0.0095):
            for x in (-0.0070, -0.0025, 0.0020, 0.0065, 0.0110):
                insert.visual(
                    Cylinder(radius=0.00065, length=0.00012),
                    origin=Origin(
                        xyz=(x, y + (-0.00037 if side == "front" else 0.00037), z),
                        rpy=(math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=soot_black,
                    name=f"{side}_hole_{idx}",
                )
                idx += 1
    insert.visual(
        Box((0.0008, 0.0098, chimney_h)),
        origin=Origin(xyz=(-0.0100, 0.0, chimney_z)),
        material=chrome,
        name="flint_side_wall",
    )
    insert.visual(
        Box((0.0008, 0.0098, chimney_h)),
        origin=Origin(xyz=(0.0130, 0.0, chimney_z)),
        material=chrome,
        name="wick_side_wall",
    )
    insert.visual(
        Cylinder(radius=0.0013, length=0.0100),
        origin=Origin(xyz=(0.0065, 0.0, lower_h + 0.00565)),
        material=wick_mat,
        name="wick",
    )
    insert.visual(
        Box((0.0040, 0.0072, 0.0062)),
        origin=Origin(xyz=(-0.0065, 0.0, lower_h + 0.0042)),
        material=chrome,
        name="striker_fork",
    )
    insert.visual(
        Cylinder(radius=0.0010, length=0.0100),
        origin=Origin(xyz=(-0.0065, 0.0, lower_h + 0.0080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="wheel_axle_boss",
    )

    model.articulation(
        "shell_to_insert",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=insert,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.028),
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.0072,
                0.0030,
                body_style="cylindrical",
                grip=KnobGrip(style="knurled", count=28, depth=0.00042, helix_angle_deg=24.0),
            ),
            "knurled_striker_wheel",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="knurled_wheel",
    )
    striker_wheel.visual(
        Cylinder(radius=0.00055, length=0.0050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="short_axle",
    )
    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(-0.0065, 0.0, lower_h + 0.0080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    wheel = object_model.get_part("striker_wheel")
    lid_joint = object_model.get_articulation("case_to_lid")
    slide_joint = object_model.get_articulation("shell_to_insert")

    ctx.expect_within(
        insert,
        lower_shell,
        axes="xy",
        elem_a="fuel_can",
        elem_b="lower_shell_cup",
        margin=0.001,
        name="insert fuel can fits inside the case footprint",
    )
    ctx.expect_overlap(
        insert,
        lower_shell,
        axes="z",
        elem_a="fuel_can",
        elem_b="lower_shell_cup",
        min_overlap=0.025,
        name="insert remains seated in the lower shell",
    )
    ctx.expect_overlap(
        wheel,
        insert,
        axes="xz",
        elem_a="knurled_wheel",
        elem_b="striker_fork",
        min_overlap=0.0025,
        name="striker wheel is cradled beside the wick opening",
    )

    closed_top = ctx.part_world_aabb(lid)
    rest_insert = ctx.part_world_position(insert)
    with ctx.pose({lid_joint: 1.45, slide_joint: 0.022}):
        opened_top = ctx.part_world_aabb(lid)
        lifted_insert = ctx.part_world_position(insert)
        ctx.expect_overlap(
            insert,
            lower_shell,
            axes="z",
            elem_a="fuel_can",
            elem_b="lower_shell_cup",
            min_overlap=0.006,
            name="lifted insert is still visibly guided by the case",
        )

    ctx.check(
        "lid rotates upward on the side hinge",
        closed_top is not None and opened_top is not None and opened_top[1][2] > closed_top[1][2] + 0.010,
        details=f"closed_aabb={closed_top}, opened_aabb={opened_top}",
    )
    ctx.check(
        "insert slides upward out of the lower shell",
        rest_insert is not None and lifted_insert is not None and lifted_insert[2] > rest_insert[2] + 0.020,
        details=f"rest={rest_insert}, lifted={lifted_insert}",
    )

    return ctx.report()


object_model = build_object_model()

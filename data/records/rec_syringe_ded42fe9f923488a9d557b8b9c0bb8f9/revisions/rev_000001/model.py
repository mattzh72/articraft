from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BARREL_LENGTH = 0.105
BARREL_OUTER_RADIUS = 0.013
BARREL_INNER_RADIUS = 0.0102
PLUNGER_TRAVEL = 0.040
BARREL_REAR_X = BARREL_LENGTH / 2.0


def _x_cylinder(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _x_origin(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_origin(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_syringe")

    clear_polycarbonate = Material("clear_polycarbonate", rgba=(0.78, 0.92, 1.0, 0.38))
    dark_ink = Material("printed_black", rgba=(0.02, 0.025, 0.03, 1.0))
    blue_polymer = Material("blue_polymer", rgba=(0.08, 0.28, 0.78, 1.0))
    soft_rubber = Material("soft_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    stainless = Material("stainless_tip", rgba=(0.72, 0.72, 0.68, 1.0))

    barrel = model.part("barrel")

    barrel_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (BARREL_OUTER_RADIUS, -BARREL_LENGTH / 2.0),
            (BARREL_OUTER_RADIUS, BARREL_LENGTH / 2.0),
        ],
        inner_profile=[
            (BARREL_INNER_RADIUS, -BARREL_LENGTH / 2.0),
            (BARREL_INNER_RADIUS, BARREL_LENGTH / 2.0),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    barrel.visual(
        mesh_from_geometry(barrel_shell, "barrel_shell"),
        material=clear_polycarbonate,
        name="barrel_shell",
    )

    nozzle_profile = [
        (0.0, -0.086),
        (0.0032, -0.086),
        (0.0032, -0.071),
        (0.0048, -0.069),
        (0.0076, -BARREL_LENGTH / 2.0),
        (0.0, -BARREL_LENGTH / 2.0),
    ]
    nozzle = LatheGeometry(nozzle_profile, segments=56, closed=True).rotate_y(math.pi / 2.0)
    barrel.visual(
        mesh_from_geometry(nozzle, "nozzle_tip"),
        material=stainless,
        name="nozzle_tip",
    )

    barrel.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=_x_origin(-BARREL_LENGTH / 2.0 + 0.001),
        material=clear_polycarbonate,
        name="front_collar",
    )
    barrel.visual(
        Cylinder(radius=0.0150, length=0.005),
        origin=_x_origin(BARREL_REAR_X + 0.001),
        material=clear_polycarbonate,
        name="rear_stop_ring",
    )
    barrel.visual(
        Box((0.006, 0.064, 0.012)),
        origin=Origin(xyz=(BARREL_REAR_X + 0.002, 0.0, 0.0)),
        material=clear_polycarbonate,
        name="finger_flange",
    )
    # Low side rails and a rear bridge form visible pull stops while keeping the
    # syringe flat enough to store in a desktop drawer.
    for side, y in enumerate((-0.018, 0.018)):
        barrel.visual(
            Box((0.050, 0.004, 0.005)),
            origin=Origin(xyz=(BARREL_REAR_X + 0.027, y, 0.0)),
            material=clear_polycarbonate,
            name=f"stop_rail_{side}",
        )
    stop_x = BARREL_REAR_X + 0.052
    barrel.visual(
        Box((0.005, 0.005, 0.011)),
        origin=Origin(xyz=(stop_x, -0.007, 0.002)),
        material=clear_polycarbonate,
        name="pull_stop_lug_0",
    )
    barrel.visual(
        Box((0.005, 0.005, 0.011)),
        origin=Origin(xyz=(stop_x, 0.007, 0.002)),
        material=clear_polycarbonate,
        name="pull_stop_lug_1",
    )
    barrel.visual(
        Box((0.005, 0.042, 0.004)),
        origin=Origin(xyz=(stop_x, 0.0, 0.009)),
        material=clear_polycarbonate,
        name="pull_stop_arch",
    )
    for side, y in enumerate((-0.018, 0.018)):
        barrel.visual(
            Box((0.005, 0.004, 0.013)),
            origin=Origin(xyz=(stop_x, y, 0.0045)),
            material=clear_polycarbonate,
            name=f"pull_stop_post_{side}",
        )

    for i, x in enumerate([-0.037, -0.027, -0.017, -0.007, 0.003, 0.013, 0.023, 0.033]):
        long_mark = i % 2 == 0
        barrel.visual(
            Box((0.0010, 0.014 if long_mark else 0.008, 0.0012)),
            origin=Origin(xyz=(x, 0.0, BARREL_OUTER_RADIUS + 0.0002)),
            material=dark_ink,
            name=f"graduation_{i}",
        )
    barrel.visual(
        Box((0.074, 0.0016, 0.0012)),
        origin=Origin(xyz=(-0.002, -0.0076, BARREL_OUTER_RADIUS + 0.0002)),
        material=dark_ink,
        name="graduation_baseline",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0096, length=0.012),
        origin=_x_origin(-0.088),
        material=soft_rubber,
        name="piston_seal",
    )
    plunger.visual(
        Cylinder(radius=0.0025, length=0.138),
        origin=_x_origin(-0.023),
        material=blue_polymer,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0067, length=0.005),
        origin=_x_origin(0.006),
        material=blue_polymer,
        name="pull_limit_collar",
    )
    plunger.visual(
        Cylinder(radius=0.0028, length=0.026),
        origin=_y_origin(0.048),
        material=blue_polymer,
        name="thumb_hinge_pin",
    )

    thumb_tab = model.part("thumb_tab")
    outer = rounded_rect_profile(0.038, 0.030, 0.006, corner_segments=7)
    inner = rounded_rect_profile(0.022, 0.014, 0.004, corner_segments=7)
    thumb_ring = ExtrudeWithHolesGeometry(outer, [inner], 0.004, center=True).translate(0.022, 0.0, 0.0)
    thumb_tab.visual(
        mesh_from_geometry(thumb_ring, "thumb_ring"),
        material=blue_polymer,
        name="thumb_ring",
    )
    thumb_tab.visual(
        Cylinder(radius=0.0034, length=0.016),
        origin=_y_origin(0.0),
        material=blue_polymer,
        name="tab_hinge_sleeve",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(BARREL_REAR_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=PLUNGER_TRAVEL),
    )
    model.articulation(
        "plunger_to_thumb_tab",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=thumb_tab,
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    thumb_tab = object_model.get_part("thumb_tab")
    slide = object_model.get_articulation("barrel_to_plunger")
    fold = object_model.get_articulation("plunger_to_thumb_tab")

    ctx.allow_overlap(
        plunger,
        thumb_tab,
        elem_a="thumb_hinge_pin",
        elem_b="tab_hinge_sleeve",
        reason="The thumb tab sleeve is intentionally captured around the plunger hinge pin.",
    )
    ctx.expect_overlap(
        plunger,
        thumb_tab,
        axes="y",
        min_overlap=0.010,
        elem_a="thumb_hinge_pin",
        elem_b="tab_hinge_sleeve",
        name="thumb sleeve captures hinge pin",
    )

    with ctx.pose({slide: 0.0, fold: 0.0}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="piston_seal",
            outer_elem="barrel_shell",
            margin=0.001,
            name="piston is coaxial in the barrel bore",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            min_overlap=0.010,
            elem_a="piston_seal",
            elem_b="barrel_shell",
            name="depressed piston remains inside barrel",
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="x",
            max_gap=0.003,
            max_penetration=0.0005,
            positive_elem="pull_limit_collar",
            negative_elem="rear_stop_ring",
            name="insertion stop collar sits at rear stop",
        )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: PLUNGER_TRAVEL, fold: 0.0}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="piston_seal",
            outer_elem="barrel_shell",
            margin=0.001,
            name="extended piston remains coaxial",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            min_overlap=0.010,
            elem_a="piston_seal",
            elem_b="barrel_shell",
            name="extended piston retains barrel insertion",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="pull_stop_lug_0",
            negative_elem="pull_limit_collar",
            name="pull stop lugs bound withdrawal",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger translates only along barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + PLUNGER_TRAVEL * 0.9
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({slide: 0.0, fold: 0.0}):
        flat_tab_aabb = ctx.part_element_world_aabb(thumb_tab, elem="thumb_ring")
    with ctx.pose({slide: 0.0, fold: math.pi / 2.0}):
        upright_tab_aabb = ctx.part_element_world_aabb(thumb_tab, elem="thumb_ring")

    ctx.check(
        "thumb tab folds up from flat stow pose",
        flat_tab_aabb is not None
        and upright_tab_aabb is not None
        and upright_tab_aabb[1][2] > flat_tab_aabb[1][2] + 0.025,
        details=f"flat={flat_tab_aabb}, upright={upright_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

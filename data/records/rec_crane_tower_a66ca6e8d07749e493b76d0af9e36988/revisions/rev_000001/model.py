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


YELLOW = Material("crane_yellow", color=(1.0, 0.74, 0.05, 1.0))
DARK_STEEL = Material("dark_steel", color=(0.09, 0.10, 0.11, 1.0))
GREY = Material("concrete_grey", color=(0.48, 0.48, 0.46, 1.0))
CAB = Material("cab_white", color=(0.86, 0.88, 0.82, 1.0))
GLASS = Material("blue_tinted_glass", color=(0.08, 0.20, 0.30, 0.82))
COUNTERWEIGHT = Material("counterweight_grey", color=(0.30, 0.32, 0.33, 1.0))


def _bar_pose_between(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, Origin]:
    """Return a box length and transform for a bar whose local +X runs from a to b."""
    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = -math.asin(dz / length)
    mid = ((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5)
    return length, Origin(xyz=mid, rpy=(0.0, pitch, yaw))


def _add_bar(part, name: str, a: tuple[float, float, float], b: tuple[float, float, float], thickness: float, material):
    length, origin = _bar_pose_between(a, b)
    part.visual(Box((length, thickness, thickness)), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hammerhead_tower_crane")

    mast = model.part("mast")
    mast.visual(Box((4.2, 4.2, 0.50)), origin=Origin(xyz=(0.0, 0.0, 0.25)), material=GREY, name="base_slab")
    mast.visual(Box((1.45, 1.45, 0.20)), origin=Origin(xyz=(0.0, 0.0, 17.90)), material=YELLOW, name="mast_cap")
    mast.visual(Cylinder(radius=0.95, length=0.18), origin=Origin(xyz=(0.0, 0.0, 18.09)), material=DARK_STEEL, name="fixed_slewing_ring")

    leg_xy = 0.55
    for ix, x in enumerate((-leg_xy, leg_xy)):
        for iy, y in enumerate((-leg_xy, leg_xy)):
            mast.visual(
                Box((0.18, 0.18, 17.30)),
                origin=Origin(xyz=(x, y, 9.15)),
                material=YELLOW,
                name=f"mast_leg_{ix}_{iy}",
            )

    for level, z in enumerate((1.2, 3.4, 5.6, 7.8, 10.0, 12.2, 14.4, 16.6)):
        mast.visual(Box((1.28, 0.11, 0.11)), origin=Origin(xyz=(0.0, -leg_xy, z)), material=YELLOW, name=f"mast_frame_front_{level}")
        mast.visual(Box((1.28, 0.11, 0.11)), origin=Origin(xyz=(0.0, leg_xy, z)), material=YELLOW, name=f"mast_frame_rear_{level}")
        mast.visual(Box((0.11, 1.28, 0.11)), origin=Origin(xyz=(-leg_xy, 0.0, z)), material=YELLOW, name=f"mast_frame_side_0_{level}")
        mast.visual(Box((0.11, 1.28, 0.11)), origin=Origin(xyz=(leg_xy, 0.0, z)), material=YELLOW, name=f"mast_frame_side_1_{level}")

    for level, z0 in enumerate((1.2, 3.4, 5.6, 7.8, 10.0, 12.2, 14.4)):
        z1 = z0 + 2.2
        _add_bar(mast, f"mast_diag_front_{level}", (-leg_xy, -leg_xy, z0), (leg_xy, -leg_xy, z1), 0.075, YELLOW)
        _add_bar(mast, f"mast_diag_rear_{level}", (leg_xy, leg_xy, z0), (-leg_xy, leg_xy, z1), 0.075, YELLOW)
        _add_bar(mast, f"mast_diag_side_0_{level}", (-leg_xy, leg_xy, z0), (-leg_xy, -leg_xy, z1), 0.075, YELLOW)
        _add_bar(mast, f"mast_diag_side_1_{level}", (leg_xy, -leg_xy, z0), (leg_xy, leg_xy, z1), 0.075, YELLOW)

    upperworks = model.part("upperworks")
    upperworks.visual(Cylinder(radius=0.83, length=0.16), origin=Origin(xyz=(0.0, 0.0, 0.08)), material=YELLOW, name="rotating_slewing_ring")
    upperworks.visual(Cylinder(radius=0.58, length=0.20), origin=Origin(xyz=(0.0, 0.0, 0.26)), material=YELLOW, name="slew_pedestal")
    upperworks.visual(Box((1.75, 1.45, 0.20)), origin=Origin(xyz=(0.05, -0.05, 0.42)), material=YELLOW, name="turntable_deck")
    upperworks.visual(Box((0.42, 0.42, 1.45)), origin=Origin(xyz=(0.0, 0.0, 1.16)), material=YELLOW, name="center_tower")

    # A compact operator cab is fixed to the rotating upperworks beside the mast center.
    upperworks.visual(Box((0.90, 0.72, 0.66)), origin=Origin(xyz=(0.45, -1.03, 0.85)), material=CAB, name="cab_box")
    upperworks.visual(Box((0.58, 0.025, 0.30)), origin=Origin(xyz=(0.43, -1.388, 0.94)), material=GLASS, name="front_window")
    upperworks.visual(Box((0.025, 0.42, 0.28)), origin=Origin(xyz=(0.91, -1.03, 0.94)), material=GLASS, name="side_window")

    # Working jib rails and a shorter counter-jib form the hammerhead silhouette.
    rail_z = 1.25
    top_z = 1.95
    truss_y = 0.66
    for side, y in enumerate((-0.35, 0.35)):
        upperworks.visual(Box((24.0, 0.12, 0.12)), origin=Origin(xyz=(12.8, y, rail_z)), material=YELLOW, name=f"jib_rail_{side}")
        upperworks.visual(Box((8.0, 0.12, 0.12)), origin=Origin(xyz=(-4.2, y, rail_z)), material=YELLOW, name=f"counter_rail_{side}")
        upperworks.visual(Box((2.0, 0.12, 0.12)), origin=Origin(xyz=(0.0, y, rail_z)), material=YELLOW, name=f"center_rail_{side}")

    upperworks.visual(Box((0.16, 1.45, 0.14)), origin=Origin(xyz=(0.0, 0.0, rail_z)), material=YELLOW, name="center_crossbeam")
    for side, y in enumerate((-truss_y, truss_y)):
        upperworks.visual(Box((24.0, 0.11, 0.11)), origin=Origin(xyz=(12.8, y, top_z)), material=YELLOW, name=f"jib_top_chord_{side}")
        upperworks.visual(Box((8.4, 0.11, 0.11)), origin=Origin(xyz=(-4.0, y, top_z)), material=YELLOW, name=f"counter_top_chord_{side}")

    for idx, x in enumerate((1.0, 5.0, 9.0, 13.0, 17.0, 21.0, 24.5)):
        upperworks.visual(Box((0.12, 1.45, 0.12)), origin=Origin(xyz=(x, 0.0, rail_z)), material=YELLOW, name=f"jib_crossbeam_{idx}")
        for side, y in enumerate((-truss_y, truss_y)):
            upperworks.visual(Box((0.10, 0.10, 0.70)), origin=Origin(xyz=(x, y, 1.60)), material=YELLOW, name=f"jib_post_{idx}_{side}")

    for idx, x in enumerate((-8.0, -5.5, -3.0, -0.5)):
        upperworks.visual(Box((0.12, 1.45, 0.12)), origin=Origin(xyz=(x, 0.0, rail_z)), material=YELLOW, name=f"counter_crossbeam_{idx}")
        for side, y in enumerate((-truss_y, truss_y)):
            upperworks.visual(Box((0.10, 0.10, 0.70)), origin=Origin(xyz=(x, y, 1.60)), material=YELLOW, name=f"counter_post_{idx}_{side}")

    # Side truss diagonals make the long jib read as a crane boom, not as a plain beam.
    for idx, x0 in enumerate((1.0, 5.0, 9.0, 13.0, 17.0, 21.0)):
        x1 = x0 + 4.0
        for side, y in enumerate((-truss_y, truss_y)):
            _add_bar(upperworks, f"jib_diag_{idx}_{side}", (x0, y, rail_z), (x1, y, top_z), 0.060, YELLOW)
    for idx, x0 in enumerate((-8.0, -5.5, -3.0)):
        x1 = x0 + 2.5
        for side, y in enumerate((-truss_y, truss_y)):
            _add_bar(upperworks, f"counter_diag_{idx}_{side}", (x0, y, rail_z), (x1, y, top_z), 0.060, YELLOW)

    upperworks.visual(Box((1.45, 1.12, 1.15)), origin=Origin(xyz=(-7.35, 0.0, 0.94)), material=COUNTERWEIGHT, name="counterweight")

    trolley = model.part("trolley")
    trolley.visual(Box((0.88, 1.08, 0.28)), origin=Origin(xyz=(0.0, 0.0, 0.22)), material=YELLOW, name="trolley_frame")
    trolley.visual(Box((0.74, 0.08, 0.44)), origin=Origin(xyz=(0.0, -0.58, 0.00)), material=YELLOW, name="side_cheek_0")
    trolley.visual(Box((0.74, 0.08, 0.44)), origin=Origin(xyz=(0.0, 0.58, 0.00)), material=YELLOW, name="side_cheek_1")
    for ix, x in enumerate((-0.28, 0.28)):
        for iy, y in enumerate((-0.35, 0.35)):
            trolley.visual(Box((0.20, 0.16, 0.08)), origin=Origin(xyz=(x, y, 0.04)), material=DARK_STEEL, name=f"wheel_{ix}_{iy}")
    trolley.visual(Box((0.34, 0.34, 0.24)), origin=Origin(xyz=(0.0, 0.0, -0.02)), material=DARK_STEEL, name="winch_box")
    trolley.visual(Cylinder(radius=0.030, length=10.0), origin=Origin(xyz=(0.0, 0.0, -5.12)), material=DARK_STEEL, name="hoist_cable")
    trolley.visual(Box((0.42, 0.24, 0.25)), origin=Origin(xyz=(0.0, 0.0, -10.22)), material=DARK_STEEL, name="hook_block")
    trolley.visual(Box((0.12, 0.08, 0.34)), origin=Origin(xyz=(0.0, 0.0, -10.50)), material=DARK_STEEL, name="hook")

    model.articulation(
        "mast_to_slew",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=upperworks,
        origin=Origin(xyz=(0.0, 0.0, 18.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "jib_to_trolley",
        ArticulationType.PRISMATIC,
        parent=upperworks,
        child=trolley,
        origin=Origin(xyz=(3.0, 0.0, 1.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.7, lower=0.0, upper=20.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    upperworks = object_model.get_part("upperworks")
    trolley = object_model.get_part("trolley")
    slew = object_model.get_articulation("mast_to_slew")
    slide = object_model.get_articulation("jib_to_trolley")

    ctx.check("slewing joint uses vertical axis", tuple(slew.axis) == (0.0, 0.0, 1.0), details=f"axis={slew.axis}")
    ctx.check("trolley slides along jib", tuple(slide.axis) == (1.0, 0.0, 0.0), details=f"axis={slide.axis}")

    with ctx.pose({slew: 0.0, slide: 0.0}):
        ctx.expect_contact(
            upperworks,
            mast,
            elem_a="rotating_slewing_ring",
            elem_b="fixed_slewing_ring",
            contact_tol=1e-5,
            name="rotating ring seats on fixed ring",
        )
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="wheel_0_0",
            elem_b="jib_rail_0",
            contact_tol=1e-5,
            name="trolley wheel rests on jib rail",
        )
        ctx.expect_overlap(
            trolley,
            upperworks,
            axes="x",
            elem_a="wheel_0_0",
            elem_b="jib_rail_0",
            min_overlap=0.15,
            name="trolley wheel is retained on rail at inner stop",
        )
        rest_pos = ctx.part_world_position(trolley)

    with ctx.pose({slide: 20.5}):
        ctx.expect_contact(
            trolley,
            upperworks,
            elem_a="wheel_0_0",
            elem_b="jib_rail_0",
            contact_tol=1e-5,
            name="trolley wheel still rides rail at outer stop",
        )
        ctx.expect_overlap(
            trolley,
            upperworks,
            axes="x",
            elem_a="wheel_0_0",
            elem_b="jib_rail_0",
            min_overlap=0.15,
            name="trolley remains on jib at outer stop",
        )
        extended_pos = ctx.part_world_position(trolley)

    ctx.check(
        "trolley extends outward along the jib",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 20.0
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()

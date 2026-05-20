from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_WIDTH = 1.10
FRAME_HEIGHT = 0.78
INNER_SPAN = 0.89
SHUTTER_COUNT = 6
SHUTTER_SPACING = 0.096
SHUTTER_Z0 = -0.240
REST_PITCH = -0.30


def _extruded_louver_blade(length: float, chord: float, name: str):
    """Build a connected cambered shutter blade extruded along local X."""
    profile_yz = [
        (-0.004, 0.000),
        (0.002, 0.009),
        (0.042, 0.014),
        (0.095, 0.012),
        (chord - 0.010, 0.005),
        (chord, 0.0015),
        (chord, -0.0015),
        (chord - 0.010, -0.005),
        (0.095, -0.010),
        (0.042, -0.012),
        (0.002, -0.008),
    ]
    geom = MeshGeometry()
    half = length / 2.0
    back_indices = []
    front_indices = []
    for y, z in profile_yz:
        back_indices.append(geom.add_vertex(-half, y, z))
    for y, z in profile_yz:
        front_indices.append(geom.add_vertex(half, y, z))

    n = len(profile_yz)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(back_indices[i], back_indices[j], front_indices[j])
        geom.add_face(back_indices[i], front_indices[j], front_indices[i])

    back_center = geom.add_vertex(-half, sum(p[0] for p in profile_yz) / n, sum(p[1] for p in profile_yz) / n)
    front_center = geom.add_vertex(half, sum(p[0] for p in profile_yz) / n, sum(p[1] for p in profile_yz) / n)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(back_center, back_indices[j], back_indices[i])
        geom.add_face(front_center, front_indices[i], front_indices[j])
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="layered_slat_shutters")

    rail_mat = model.material("dark_anodized_rail", rgba=(0.08, 0.09, 0.10, 1.0))
    bearing_mat = model.material("black_bearing_bushings", rgba=(0.015, 0.015, 0.014, 1.0))
    shaft_mat = model.material("brushed_steel_shafts", rgba=(0.65, 0.66, 0.62, 1.0))
    blade_mat = model.material("warm_grey_shutters", rgba=(0.47, 0.50, 0.49, 1.0))

    frame = model.part("frame")
    side_x = FRAME_WIDTH / 2.0 - 0.035
    for side_index, x in enumerate((-side_x, side_x)):
        frame.visual(
            Box((0.070, 0.120, FRAME_HEIGHT)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=rail_mat,
            name=f"side_rail_{side_index}",
        )
    for bar_index, z in enumerate((-FRAME_HEIGHT / 2.0 + 0.0225, FRAME_HEIGHT / 2.0 - 0.0225)):
        frame.visual(
            Box((FRAME_WIDTH, 0.100, 0.045)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rail_mat,
            name=f"cross_rail_{bar_index}",
        )

    # Bearing cups are fixed to the inner faces of the side rails.  The shutter
    # shafts terminate against them, making each louver read as mechanically
    # captured without hiding the pivots inside the rails.
    boss_inner = INNER_SPAN / 2.0
    boss_len = 0.040
    boss_centers = (-(boss_inner + boss_len / 2.0), boss_inner + boss_len / 2.0)
    shutter_zs = [SHUTTER_Z0 + i * SHUTTER_SPACING for i in range(SHUTTER_COUNT)]
    for shutter_index, z in enumerate(shutter_zs):
        for side_index, x in enumerate(boss_centers):
            frame.visual(
                Cylinder(radius=0.023, length=boss_len),
                origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bearing_mat,
                name=f"bearing_{shutter_index}_{side_index}",
            )

    blade_mesh = _extruded_louver_blade(0.860, 0.155, "cambered_louver_blade")
    for shutter_index, z in enumerate(shutter_zs):
        shutter = model.part(f"shutter_{shutter_index}")
        shutter.visual(
            Cylinder(radius=0.011, length=INNER_SPAN),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shaft_mat,
            name="shaft",
        )
        shutter.visual(
            blade_mesh,
            material=blade_mat,
            name="blade",
        )
        model.articulation(
            f"frame_to_shutter_{shutter_index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=shutter,
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(REST_PITCH, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-0.70, upper=0.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    shutters = [object_model.get_part(f"shutter_{i}") for i in range(SHUTTER_COUNT)]
    joints = [object_model.get_articulation(f"frame_to_shutter_{i}") for i in range(SHUTTER_COUNT)]

    ctx.check("six independent shutters", len(shutters) == SHUTTER_COUNT and len(joints) == SHUTTER_COUNT)

    for i, shutter in enumerate(shutters):
        ctx.expect_contact(
            shutter,
            frame,
            elem_a="shaft",
            elem_b=f"bearing_{i}_0",
            contact_tol=0.0008,
            name=f"shutter {i} shaft seats in bearing 0",
        )
        ctx.expect_contact(
            shutter,
            frame,
            elem_a="shaft",
            elem_b=f"bearing_{i}_1",
            contact_tol=0.0008,
            name=f"shutter {i} shaft seats in bearing 1",
        )

    for i in range(1, SHUTTER_COUNT):
        ctx.expect_origin_gap(
            shutters[i],
            shutters[i - 1],
            axis="z",
            min_gap=SHUTTER_SPACING - 0.001,
            max_gap=SHUTTER_SPACING + 0.001,
            name=f"shutter {i} hinge is evenly stacked",
        )
        ctx.expect_overlap(
            shutters[i],
            shutters[i - 1],
            axes="y",
            elem_a="blade",
            elem_b="blade",
            min_overlap=0.075,
            name=f"shutter {i} overlaps the neighboring louver layer",
        )

    def blade_center_z(shutter):
        bounds = ctx.part_element_world_aabb(shutter, elem="blade")
        if bounds is None:
            return None
        low, high = bounds
        return (low[2] + high[2]) / 2.0

    rest_up = blade_center_z(shutters[2])
    rest_down = blade_center_z(shutters[3])
    with ctx.pose({joints[2]: 0.55, joints[3]: -0.55}):
        moved_up = blade_center_z(shutters[2])
        moved_down = blade_center_z(shutters[3])

    ctx.check(
        "adjacent shutters pivot independently",
        rest_up is not None
        and rest_down is not None
        and moved_up is not None
        and moved_down is not None
        and moved_up > rest_up + 0.025
        and moved_down < rest_down - 0.025,
        details=f"rest_up={rest_up}, moved_up={moved_up}, rest_down={rest_down}, moved_down={moved_down}",
    )

    return ctx.report()


object_model = build_object_model()

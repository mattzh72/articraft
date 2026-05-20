from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


NUM_BLADES = 5
RAIL_LENGTH = 0.96
RAIL_DEPTH = 0.09
RAIL_HEIGHT = 0.065
PIVOT_SPACING = 0.17
PIVOT_Z = -0.010

SEAT_RADIUS = 0.018
SEAT_THICKNESS = 0.010
WASHER_RADIUS = 0.015
WASHER_THICKNESS = 0.004

BLADE_WIDTH = 0.118
BLADE_THICKNESS = 0.018
BLADE_BODY_HEIGHT = 0.72
BLADE_BODY_TOP_Z = -0.095

JOINT_LOWER = -1.15
JOINT_UPPER = 1.15


def pivot_positions() -> list[float]:
    half = (NUM_BLADES - 1) / 2.0
    return [(i - half) * PIVOT_SPACING for i in range(NUM_BLADES)]


def make_support_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, RAIL_HEIGHT / 2.0))
    )

    front_lip = cq.Workplane("XY").box(RAIL_LENGTH - 0.02, 0.014, 0.020).translate(
        (0.0, RAIL_DEPTH / 2.0 - 0.007, 0.013)
    )
    rear_return = cq.Workplane("XY").box(RAIL_LENGTH - 0.03, 0.012, 0.018).translate(
        (0.0, -RAIL_DEPTH / 2.0 + 0.006, 0.012)
    )

    shell = body.union(front_lip).union(rear_return)

    for x in pivot_positions():
        front_cheek = cq.Workplane("XY").box(0.052, 0.014, 0.024).translate(
            (x, 0.025, -0.011)
        )
        rear_cheek = cq.Workplane("XY").box(0.052, 0.014, 0.024).translate(
            (x, -0.025, -0.011)
        )
        shell = shell.union(front_cheek).union(rear_cheek)

    return shell


def make_blade_shell() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.010).extrude(-0.028)

    stalk = cq.Workplane("XY").box(0.020, 0.026, 0.060).translate((0.0, 0.0, -0.054))
    fairing = cq.Workplane("XY").box(0.024, 0.060, 0.056).translate((0.0, 0.0, -0.092))

    blade_body = (
        cq.Workplane("XY")
        .ellipse(BLADE_THICKNESS / 2.0, BLADE_WIDTH / 2.0)
        .extrude(-BLADE_BODY_HEIGHT)
        .translate((0.0, 0.0, BLADE_BODY_TOP_Z))
    )

    return hub.union(stalk).union(fairing).union(blade_body)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_shutter_blade_assembly")

    support_finish = model.material("support_finish", color=(0.23, 0.25, 0.28))
    seat_finish = model.material("seat_finish", color=(0.08, 0.09, 0.10))
    blade_finish = model.material("blade_finish", color=(0.73, 0.75, 0.78))
    hub_finish = model.material("hub_finish", color=(0.12, 0.13, 0.14))

    support_shell = mesh_from_cadquery(make_support_shell(), "support_shell")
    blade_shell = mesh_from_cadquery(make_blade_shell(), "blade_shell")

    support = model.part("support")
    support.visual(support_shell, material=support_finish, name="support_shell")

    seat_center_z = PIVOT_Z + (WASHER_THICKNESS / 2.0) + (SEAT_THICKNESS / 2.0)
    for index, x in enumerate(pivot_positions(), start=1):
        support.visual(
            Cylinder(radius=SEAT_RADIUS, length=SEAT_THICKNESS),
            origin=Origin(xyz=(x, 0.0, seat_center_z)),
            material=seat_finish,
            name=f"seat_{index}",
        )

        blade = model.part(f"blade_{index}")
        blade.visual(blade_shell, material=blade_finish, name="blade_shell")
        blade.visual(
            Cylinder(radius=WASHER_RADIUS, length=WASHER_THICKNESS),
            origin=Origin(),
            material=hub_finish,
            name="pivot_washer",
        )

        model.articulation(
            f"support_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=blade,
            origin=Origin(xyz=(x, 0.0, PIVOT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.5,
                lower=JOINT_LOWER,
                upper=JOINT_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    blades = [object_model.get_part(f"blade_{index}") for index in range(1, NUM_BLADES + 1)]
    joints = [
        object_model.get_articulation(f"support_to_blade_{index}")
        for index in range(1, NUM_BLADES + 1)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for index, blade in enumerate(blades, start=1):
        ctx.expect_contact(
            blade,
            support,
            elem_a="pivot_washer",
            elem_b=f"seat_{index}",
            name=f"blade_{index}_pivot_contacts_support",
        )
        ctx.expect_origin_gap(
            support,
            blade,
            axis="z",
            min_gap=0.009,
            max_gap=0.011,
            name=f"blade_{index}_hangs_below_support",
        )

    blade_1 = blades[0]
    joint_1 = joints[0]
    with ctx.pose({joint_1: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(blade_1, elem="blade_shell")
    with ctx.pose({joint_1: 1.0}):
        opened_aabb = ctx.part_element_world_aabb(blade_1, elem="blade_shell")

    if closed_aabb is None or opened_aabb is None:
        ctx.fail(
            "blade_1_pose_measurement_available",
            "Could not resolve blade_1 blade_shell AABB in closed and opened poses.",
        )
    else:
        closed_dx = closed_aabb[1][0] - closed_aabb[0][0]
        closed_dy = closed_aabb[1][1] - closed_aabb[0][1]
        closed_dz = closed_aabb[1][2] - closed_aabb[0][2]
        opened_dx = opened_aabb[1][0] - opened_aabb[0][0]
        opened_dy = opened_aabb[1][1] - opened_aabb[0][1]
        opened_dz = opened_aabb[1][2] - opened_aabb[0][2]

        ctx.check(
            "blade_1_rotates_about_vertical_axis",
            opened_dx > closed_dx + 0.03
            and opened_dy < closed_dy - 0.02
            and abs(opened_dz - closed_dz) < 0.01,
            details=(
                f"closed spans=({closed_dx:.3f}, {closed_dy:.3f}, {closed_dz:.3f}), "
                f"opened spans=({opened_dx:.3f}, {opened_dy:.3f}, {opened_dz:.3f})"
            ),
        )

    staggered_pose = {
        joints[0]: 0.90,
        joints[1]: -0.60,
        joints[2]: 0.40,
        joints[3]: -0.85,
        joints[4]: 0.70,
    }
    with ctx.pose(staggered_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="staggered_pose_overlap_free")
        ctx.expect_contact(
            blades[2],
            support,
            elem_a="pivot_washer",
            elem_b="seat_3",
            name="center_blade_remains_supported_in_staggered_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

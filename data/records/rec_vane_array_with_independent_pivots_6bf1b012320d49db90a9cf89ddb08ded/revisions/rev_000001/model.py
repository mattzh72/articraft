from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 1.00
FRAME_HEIGHT = 1.10
FRAME_DEPTH = 0.09
STILE_WIDTH = 0.06
RAIL_HEIGHT = 0.06

APERTURE_WIDTH = FRAME_WIDTH - (2.0 * STILE_WIDTH)
APERTURE_HEIGHT = FRAME_HEIGHT - (2.0 * RAIL_HEIGHT)

VANE_COUNT = 11
VANE_CHORD = 0.066
VANE_THICKNESS = 0.012
PIN_RADIUS = 0.005
JOURNAL_LENGTH = 0.012
BRACKET_PROJECTION = 0.022
BRACKET_WIDTH = 0.030
BRACKET_HEIGHT = 0.024
PIVOT_CLEARANCE = 0.0
BLADE_SPAN = APERTURE_WIDTH - (
    2.0 * (BRACKET_PROJECTION + JOURNAL_LENGTH + PIVOT_CLEARANCE)
)
VANE_PITCH = APERTURE_HEIGHT / (VANE_COUNT + 1)


def vane_center_z(index: int) -> float:
    return (-APERTURE_HEIGHT / 2.0) + (VANE_PITCH * (index + 1))


def make_vane_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("YZ")
        .ellipse(VANE_CHORD / 2.0, VANE_THICKNESS / 2.0)
        .extrude(BLADE_SPAN)
        .translate((-BLADE_SPAN / 2.0, 0.0, 0.0))
    )
    left_pin = (
        cq.Workplane("YZ", origin=(-BLADE_SPAN / 2.0, 0.0, 0.0))
        .circle(PIN_RADIUS)
        .extrude(-JOURNAL_LENGTH)
    )
    right_pin = (
        cq.Workplane("YZ", origin=(BLADE_SPAN / 2.0, 0.0, 0.0))
        .circle(PIN_RADIUS)
        .extrude(JOURNAL_LENGTH)
    )
    return blade.union(left_pin).union(right_pin)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vane_array")

    frame_material = model.material("frame_powdercoat", rgba=(0.20, 0.22, 0.24, 1.0))
    vane_material = model.material("vane_satin_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-(FRAME_WIDTH - STILE_WIDTH) / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="left_stile",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=((FRAME_WIDTH - STILE_WIDTH) / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="right_stile",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, (FRAME_HEIGHT - RAIL_HEIGHT) / 2.0)),
        material=frame_material,
        name="top_rail",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -(FRAME_HEIGHT - RAIL_HEIGHT) / 2.0)),
        material=frame_material,
        name="bottom_rail",
    )

    for index in range(VANE_COUNT):
        z_pos = vane_center_z(index)
        frame.visual(
            Box((BRACKET_PROJECTION, BRACKET_WIDTH, BRACKET_HEIGHT)),
            origin=Origin(
                xyz=(
                    (-APERTURE_WIDTH / 2.0) + (BRACKET_PROJECTION / 2.0),
                    0.0,
                    z_pos,
                )
            ),
            material=frame_material,
            name=f"left_bracket_{index + 1}",
        )
        frame.visual(
            Box((BRACKET_PROJECTION, BRACKET_WIDTH, BRACKET_HEIGHT)),
            origin=Origin(
                xyz=(
                    (APERTURE_WIDTH / 2.0) - (BRACKET_PROJECTION / 2.0),
                    0.0,
                    z_pos,
                )
            ),
            material=frame_material,
            name=f"right_bracket_{index + 1}",
        )

        vane_name = f"vane_{index + 1}"
        joint_name = f"frame_to_vane_{index + 1}"
        vane = model.part(vane_name)
        vane.visual(
            mesh_from_cadquery(make_vane_shape(), f"{vane_name}_blade"),
            material=vane_material,
            name="blade",
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.5,
                lower=-1.0,
                upper=1.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    vanes = [object_model.get_part(f"vane_{index + 1}") for index in range(VANE_COUNT)]
    joints = [
        object_model.get_articulation(f"frame_to_vane_{index + 1}")
        for index in range(VANE_COUNT)
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

    for joint in joints:
        axis = joint.axis
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_long_axis",
            abs(axis[0]) > 0.999999 and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9,
            details=f"expected pure x-axis pivot, got {axis}",
        )
        ctx.check(
            f"{joint.name}_symmetric_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"expected opening limits on both sides of zero, got {limits}",
        )

    for vane in vanes:
        ctx.expect_contact(vane, frame, name=f"{vane.name}_pivot_contact")
        ctx.expect_within(vane, frame, axes="xy", name=f"{vane.name}_inside_frame_span")

    for lower_vane, upper_vane in zip(vanes[:-1], vanes[1:]):
        ctx.expect_origin_gap(
            upper_vane,
            lower_vane,
            axis="z",
            min_gap=VANE_PITCH * 0.99,
            max_gap=VANE_PITCH * 1.01,
            name=f"{lower_vane.name}_to_{upper_vane.name}_uniform_pitch",
        )
        ctx.expect_gap(
            upper_vane,
            lower_vane,
            axis="z",
            min_gap=0.020,
            name=f"{lower_vane.name}_to_{upper_vane.name}_vertical_clearance",
        )
        ctx.expect_overlap(
            upper_vane,
            lower_vane,
            axes="x",
            min_overlap=0.82,
            name=f"{lower_vane.name}_to_{upper_vane.name}_shared_span",
        )

    staggered_pose = {
        joint: (0.85 if index % 2 == 0 else -0.85)
        for index, joint in enumerate(joints)
    }
    with ctx.pose(staggered_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="staggered_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

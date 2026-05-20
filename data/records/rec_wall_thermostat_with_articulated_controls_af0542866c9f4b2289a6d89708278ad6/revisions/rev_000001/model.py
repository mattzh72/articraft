from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wall_thermostat")

    def circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos((2.0 * math.pi * index) / segments),
                radius * math.sin((2.0 * math.pi * index) / segments),
            )
            for index in range(segments)
        ]

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.79, 0.81, 0.83, 1.0))
    dial_face = model.material("dial_face", rgba=(0.90, 0.91, 0.92, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body_outer = rounded_rect_profile(0.094, 0.094, 0.018, corner_segments=8)
    dial_clearance = circle_profile(0.0305, segments=56)
    body_shell_mesh = save_mesh(
        "thermostat_body_shell",
        ExtrudeWithHolesGeometry(
            body_outer,
            [dial_clearance],
            0.024,
            center=True,
        ),
    )
    shaft_boss_mesh = save_mesh(
        "thermostat_shaft_boss",
        ExtrudeWithHolesGeometry(
            circle_profile(0.0105, segments=40),
            [circle_profile(0.0078, segments=40)],
            0.002,
            center=True,
        ),
    )
    stand_plate_mesh = save_mesh(
        "thermostat_stand_plate",
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.064, 0.028, 0.006, corner_segments=6),
            0.0025,
            cap=True,
            closed=True,
        ),
    )

    body = model.part("body")
    body.visual(body_shell_mesh, material=body_white, name="body_shell")
    body.visual(
        shaft_boss_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=bezel_gray,
        name="shaft_boss",
    )
    for name, xyz in (
        ("rib_upper_left", (-0.018, 0.008, -0.0068)),
        ("rib_upper_right", (0.018, 0.008, -0.0068)),
        ("rib_lower_left", (-0.018, -0.008, -0.0068)),
        ("rib_lower_right", (0.018, -0.008, -0.0068)),
    ):
        body.visual(
            Box((0.024, 0.004, 0.0024)),
            origin=Origin(xyz=xyz),
            material=bezel_gray,
            name=name,
        )
    body.visual(
        Box((0.015, 0.010, 0.006)),
        origin=Origin(xyz=(-0.027, -0.036, -0.012)),
        material=bezel_gray,
        name="left_hinge_mount",
    )
    body.visual(
        Box((0.015, 0.010, 0.006)),
        origin=Origin(xyz=(0.027, -0.036, -0.012)),
        material=bezel_gray,
        name="right_hinge_mount",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(-0.027, -0.034, -0.0165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.027, -0.034, -0.0165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.095, 0.095, 0.032)),
        mass=0.22,
        origin=Origin(),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.0285, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dial_silver,
        name="grip_ring",
    )
    dial.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dial_face,
        name="face_cap",
    )
    dial.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dark_graphite,
        name="shaft",
    )
    dial.visual(
        Cylinder(radius=0.014, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, -0.01025)),
        material=dark_graphite,
        name="retainer",
    )
    dial.visual(
        Box((0.0025, 0.011, 0.0012)),
        origin=Origin(xyz=(0.0, 0.018, 0.0156)),
        material=dark_graphite,
        name="index_tick",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0285, length=0.025),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
    )

    stand = model.part("stand")
    stand.visual(
        stand_plate_mesh,
        origin=Origin(xyz=(0.0, 0.043, 0.00325)),
        material=dark_graphite,
        name="stand_plate",
    )
    stand.visual(
        Cylinder(radius=0.0045, length=0.042),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="hinge_barrel",
    )
    stand.visual(
        Box((0.006, 0.030, 0.004)),
        origin=Origin(xyz=(-0.017, 0.015, 0.002)),
        material=dark_graphite,
        name="left_stand_arm",
    )
    stand.visual(
        Box((0.006, 0.030, 0.004)),
        origin=Origin(xyz=(0.017, 0.015, 0.002)),
        material=dark_graphite,
        name="right_stand_arm",
    )
    stand.visual(
        Box((0.044, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.056, 0.0025)),
        material=rubber_black,
        name="foot_pad",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.064, 0.060, 0.006)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.030, 0.0025)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=6.0,
        ),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.034, -0.0165)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    dial_joint = object_model.get_articulation("body_to_dial")
    stand_joint = object_model.get_articulation("body_to_stand")

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
    ctx.expect_contact(
        dial,
        body,
        elem_a="retainer",
        elem_b="shaft_boss",
        name="dial_retainer_captures_body_boss",
    )
    ctx.expect_within(
        dial,
        body,
        axes="xy",
        margin=0.0,
        name="dial_stays_within_compact_body_footprint",
    )
    ctx.expect_gap(
        body,
        stand,
        axis="z",
        positive_elem="body_shell",
        negative_elem="stand_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="stand_folds_flat_to_back",
    )
    ctx.expect_contact(
        stand,
        body,
        elem_a="hinge_barrel",
        elem_b="left_hinge_barrel",
        name="stand_hinge_remains_captured",
    )
    ctx.check(
        "dial_joint_is_center_axis_rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in dial_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected continuous center-axis dial, got type={dial_joint.articulation_type} axis={dial_joint.axis}",
    )

    with ctx.pose({stand_joint: 1.05}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_stand_pose_clearances")
        ctx.expect_gap(
            body,
            stand,
            axis="z",
            positive_elem="body_shell",
            negative_elem="stand_plate",
            min_gap=0.010,
            name="stand_swings_clear_of_body_when_open",
        )
        ctx.expect_contact(
            stand,
            body,
            elem_a="hinge_barrel",
            elem_b="left_hinge_barrel",
            name="open_stand_keeps_mechanical_hinge_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

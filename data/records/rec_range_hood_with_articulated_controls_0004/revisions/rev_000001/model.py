from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.55, 0.57, 0.60, 1.0))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.21, 0.22, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.52
    canopy_height = 0.16
    shell_thickness = 0.014

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((canopy_width, canopy_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - shell_thickness / 2.0)),
        material=stainless,
        name="canopy_top",
    )
    hood_body.visual(
        Box((shell_thickness, canopy_depth, canopy_height - shell_thickness)),
        origin=Origin(
            xyz=(
                -canopy_width / 2.0 + shell_thickness / 2.0,
                0.0,
                (canopy_height - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="left_side",
    )
    hood_body.visual(
        Box((shell_thickness, canopy_depth, canopy_height - shell_thickness)),
        origin=Origin(
            xyz=(
                canopy_width / 2.0 - shell_thickness / 2.0,
                0.0,
                (canopy_height - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="right_side",
    )
    hood_body.visual(
        Box((canopy_width - 2.0 * shell_thickness, shell_thickness, canopy_height - shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                canopy_depth / 2.0 - shell_thickness / 2.0,
                (canopy_height - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="rear_panel",
    )
    hood_body.visual(
        Box((canopy_width - 2.0 * shell_thickness, shell_thickness, 0.110)),
        origin=Origin(
            xyz=(
                0.0,
                -canopy_depth / 2.0 + shell_thickness / 2.0,
                0.055,
            )
        ),
        material=stainless,
        name="front_panel",
    )
    hood_body.visual(
        Box((0.520, 0.008, 0.052)),
        origin=Origin(
            xyz=(
                0.0,
                -canopy_depth / 2.0 - 0.004,
                0.072,
            )
        ),
        material=dark_stainless,
        name="control_strip",
    )
    hood_body.visual(
        Box((0.340, 0.280, 0.024)),
        origin=Origin(xyz=(0.0, 0.090, canopy_height + 0.012)),
        material=stainless,
        name="chimney_base",
    )
    hood_body.visual(
        Box((0.320, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.040, 0.019)),
        material=charcoal,
        name="underside_front_rail",
    )
    hood_body.visual(
        Box((0.320, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.090, 0.019)),
        material=charcoal,
        name="underside_rear_rail",
    )
    hood_body.visual(
        Box((0.018, 0.126, 0.010)),
        origin=Origin(xyz=(-0.160, 0.025, 0.019)),
        material=charcoal,
        name="underside_left_rail",
    )
    hood_body.visual(
        Box((0.018, 0.126, 0.010)),
        origin=Origin(xyz=(0.160, 0.025, 0.019)),
        material=charcoal,
        name="underside_right_rail",
    )
    hood_body.visual(
        Box((0.298, 0.124, 0.010)),
        origin=Origin(xyz=(-0.151, 0.025, 0.009)),
        material=dark_stainless,
        name="left_filter",
    )
    hood_body.visual(
        Box((0.298, 0.124, 0.010)),
        origin=Origin(xyz=(0.151, 0.025, 0.009)),
        material=dark_stainless,
        name="right_filter",
    )
    hood_body.visual(
        Box((0.340, 0.220, 0.122)),
        origin=Origin(xyz=(0.0, 0.025, 0.085)),
        material=charcoal,
        name="blower_housing",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height + 0.024)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + 0.024) / 2.0)),
    )

    chimney_cover = model.part("chimney_cover")
    chimney_width = 0.300
    chimney_depth = 0.240
    chimney_height = 0.780
    chimney_wall = 0.012
    chimney_cover.visual(
        Box((chimney_width, chimney_depth, chimney_wall)),
        origin=Origin(xyz=(0.0, 0.0, chimney_height - chimney_wall / 2.0)),
        material=stainless,
        name="chimney_top",
    )
    chimney_cover.visual(
        Box((chimney_wall, chimney_depth, chimney_height - chimney_wall)),
        origin=Origin(
            xyz=(
                -chimney_width / 2.0 + chimney_wall / 2.0,
                0.0,
                (chimney_height - chimney_wall) / 2.0,
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    chimney_cover.visual(
        Box((chimney_wall, chimney_depth, chimney_height - chimney_wall)),
        origin=Origin(
            xyz=(
                chimney_width / 2.0 - chimney_wall / 2.0,
                0.0,
                (chimney_height - chimney_wall) / 2.0,
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    chimney_cover.visual(
        Box((chimney_width - 2.0 * chimney_wall, chimney_wall, chimney_height - chimney_wall)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_depth / 2.0 - chimney_wall / 2.0,
                (chimney_height - chimney_wall) / 2.0,
            )
        ),
        material=stainless,
        name="chimney_rear",
    )
    chimney_cover.visual(
        Box((chimney_width - 2.0 * chimney_wall, chimney_wall, chimney_height - chimney_wall)),
        origin=Origin(
            xyz=(
                0.0,
                -chimney_depth / 2.0 + chimney_wall / 2.0,
                (chimney_height - chimney_wall) / 2.0,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    chimney_cover.inertial = Inertial.from_geometry(
        Box((chimney_width, chimney_depth, chimney_height)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, chimney_height / 2.0)),
    )
    model.articulation(
        "hood_to_chimney_cover",
        ArticulationType.FIXED,
        parent=hood_body,
        child=chimney_cover,
        origin=Origin(xyz=(0.0, 0.090, canopy_height + 0.024)),
    )

    knob_specs = [
        ("left_knob", -0.110),
        ("center_knob", 0.0),
        ("right_knob", 0.110),
    ]
    control_face_y = -canopy_depth / 2.0 - 0.008
    control_center_z = 0.072

    for knob_name, x_pos in knob_specs:
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="mount_flange",
        )
        knob.visual(
            Cylinder(radius=0.018, length=0.022),
            origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="dial",
        )
        knob.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(0.0, -0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="center_cap",
        )
        knob.visual(
            Box((0.004, 0.003, 0.012)),
            origin=Origin(xyz=(0.0, -0.0285, 0.011,)),
            material=stainless,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.020, length=0.030),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"hood_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x_pos, control_face_y, control_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    hood_body = object_model.get_part("hood_body")
    chimney_cover = object_model.get_part("chimney_cover")
    left_knob = object_model.get_part("left_knob")
    center_knob = object_model.get_part("center_knob")
    right_knob = object_model.get_part("right_knob")

    hood_to_chimney_cover = object_model.get_articulation("hood_to_chimney_cover")
    hood_to_left_knob = object_model.get_articulation("hood_to_left_knob")
    hood_to_center_knob = object_model.get_articulation("hood_to_center_knob")
    hood_to_right_knob = object_model.get_articulation("hood_to_right_knob")

    ctx.expect_contact(chimney_cover, hood_body)
    ctx.expect_gap(
        chimney_cover,
        hood_body,
        axis="z",
        max_gap=1e-6,
        max_penetration=0.0,
        name="chimney_sits_on_canopy_top",
    )
    ctx.expect_within(chimney_cover, hood_body, axes="xy", name="chimney_centered_over_canopy")

    for knob in (left_knob, center_knob, right_knob):
        ctx.expect_contact(knob, hood_body, name=f"{knob.name}_touches_control_strip")
        ctx.expect_gap(
            hood_body,
            knob,
            axis="y",
            max_gap=0.0,
            max_penetration=0.0,
            name=f"{knob.name}_flush_to_front_face",
        )
        ctx.expect_overlap(
            knob,
            hood_body,
            axes="xz",
            min_overlap=0.012,
            name=f"{knob.name}_projects_over_control_strip",
        )

    for joint in (hood_to_left_knob, hood_to_center_knob, hood_to_right_knob):
        ctx.check(
            f"{joint.name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} should be continuous, got {joint.articulation_type!r}",
        )
        axis = tuple(round(value, 6) for value in joint.axis)
        ctx.check(
            f"{joint.name}_front_to_back_axis",
            axis == (0.0, 1.0, 0.0),
            details=f"{joint.name} axis should be (0, 1, 0), got {joint.axis!r}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_has_unbounded_motion_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"{joint.name} should be continuous without lower/upper bounds",
        )

    left_pos = ctx.part_world_position(left_knob)
    center_pos = ctx.part_world_position(center_knob)
    right_pos = ctx.part_world_position(right_knob)
    hood_pos = ctx.part_world_position(hood_body)
    assert left_pos is not None
    assert center_pos is not None
    assert right_pos is not None
    assert hood_pos is not None

    left_spacing = center_pos[0] - left_pos[0]
    right_spacing = right_pos[0] - center_pos[0]
    ctx.check(
        "knobs_evenly_spaced",
        abs(left_spacing - right_spacing) <= 0.002,
        details=f"left spacing {left_spacing:.4f} m vs right spacing {right_spacing:.4f} m",
    )
    ctx.check(
        "center_knob_centered",
        abs(center_pos[0] - hood_pos[0]) <= 0.002,
        details=f"center knob x={center_pos[0]:.4f} should align to hood center x={hood_pos[0]:.4f}",
    )
    ctx.check(
        "knobs_share_height",
        abs(left_pos[2] - center_pos[2]) <= 0.001 and abs(right_pos[2] - center_pos[2]) <= 0.001,
        details=f"knob heights are left={left_pos[2]:.4f}, center={center_pos[2]:.4f}, right={right_pos[2]:.4f}",
    )
    ctx.check(
        "only_three_moving_joints",
        len(
            [
                joint
                for joint in object_model.articulations
                if joint.articulation_type != ArticulationType.FIXED
            ]
        )
        == 3,
        details="Range hood should only articulate at the three rotary knobs.",
    )
    ctx.check(
        "chimney_mount_is_fixed",
        hood_to_chimney_cover.articulation_type == ArticulationType.FIXED,
        details="Upper chimney cover should be rigidly mounted to the hood body.",
    )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    with ctx.pose(
        {
            hood_to_left_knob: math.pi / 2.0,
            hood_to_center_knob: math.pi,
            hood_to_right_knob: -math.pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")
        ctx.expect_contact(left_knob, hood_body, name="left_knob_contact_when_rotated")
        ctx.expect_contact(center_knob, hood_body, name="center_knob_contact_when_rotated")
        ctx.expect_contact(right_knob, hood_body, name="right_knob_contact_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    place_on_face,
    proud_for_flush_mount,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    appliance_white = model.material("appliance_white", rgba=(0.93, 0.93, 0.90, 1.0))
    underside_shadow = model.material("underside_shadow", rgba=(0.75, 0.76, 0.74, 1.0))
    filter_grey = model.material("filter_grey", rgba=(0.42, 0.45, 0.47, 1.0))
    knob_charcoal = model.material("knob_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    pointer_grey = model.material("pointer_grey", rgba=(0.72, 0.74, 0.76, 1.0))

    band_width = 0.76
    band_depth = 0.03
    band_height = 0.11

    shell_width = 0.72
    shell_depth = 0.31
    shell_front_y = band_depth
    shell_back_y = shell_front_y + shell_depth
    top_thickness = 0.015
    top_z = 0.1125
    panel_bottom_z = 0.02
    panel_height = 0.087
    panel_thickness = 0.012

    hood_shell = model.part("hood_shell")
    hood_shell.visual(
        Box((shell_width, shell_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                shell_front_y + shell_depth * 0.5,
                top_z,
            )
        ),
        material=appliance_white,
        name="top_panel",
    )
    hood_shell.visual(
        Box((panel_thickness, shell_depth, panel_height)),
        origin=Origin(
            xyz=(
                -shell_width * 0.5 + panel_thickness * 0.5,
                shell_front_y + shell_depth * 0.5,
                panel_bottom_z + panel_height * 0.5,
            )
        ),
        material=underside_shadow,
        name="left_capture_side",
    )
    hood_shell.visual(
        Box((panel_thickness, shell_depth, panel_height)),
        origin=Origin(
            xyz=(
                shell_width * 0.5 - panel_thickness * 0.5,
                shell_front_y + shell_depth * 0.5,
                panel_bottom_z + panel_height * 0.5,
            )
        ),
        material=underside_shadow,
        name="right_capture_side",
    )
    hood_shell.visual(
        Box((shell_width - 2.0 * panel_thickness, panel_thickness, panel_height)),
        origin=Origin(
            xyz=(
                0.0,
                shell_back_y - panel_thickness * 0.5,
                panel_bottom_z + panel_height * 0.5,
            )
        ),
        material=underside_shadow,
        name="rear_capture_wall",
    )
    hood_shell.inertial = Inertial.from_geometry(
        Box((shell_width, shell_depth, 0.10)),
        mass=7.5,
        origin=Origin(xyz=(0.0, shell_front_y + shell_depth * 0.5, 0.07)),
    )

    front_band = model.part("front_band")
    front_band.visual(
        Box((band_width, band_depth, band_height)),
        material=appliance_white,
        name="front_panel",
    )
    front_band.inertial = Inertial.from_geometry(
        Box((band_width, band_depth, band_height)),
        mass=1.8,
    )
    model.articulation(
        "shell_to_front_band",
        ArticulationType.FIXED,
        parent=hood_shell,
        child=front_band,
        origin=Origin(xyz=(0.0, band_depth * 0.5, band_height * 0.5 + 0.01)),
    )

    filter_panel = model.part("filter_panel")
    filter_panel.visual(
        Box((0.64, 0.23, 0.004)),
        material=filter_grey,
        name="grease_filter",
    )
    filter_panel.inertial = Inertial.from_geometry(
        Box((0.64, 0.23, 0.004)),
        mass=0.8,
    )
    model.articulation(
        "shell_to_filter_panel",
        ArticulationType.FIXED,
        parent=hood_shell,
        child=filter_panel,
        origin=Origin(xyz=(0.0, 0.20, 0.103)),
    )

    knob_positions = {
        "top": (0.0, 0.015),
        "left": (-0.019, 0.0),
        "right": (0.019, 0.0),
        "bottom": (0.0, -0.015),
    }

    for label, (x_pos, z_pos) in knob_positions.items():
        knob = model.part(f"knob_{label}")
        knob.visual(
            Cylinder(radius=0.011, length=0.018),
            origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
            material=knob_charcoal,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.002, 0.008)),
            origin=Origin(xyz=(0.0, -0.008, 0.0065)),
            material=pointer_grey,
            name="pointer_ridge",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.011, length=0.018),
            mass=0.05,
            origin=Origin(),
        )

        model.articulation(
            f"front_band_to_knob_{label}",
            ArticulationType.CONTINUOUS,
            parent=front_band,
            child=knob,
            origin=place_on_face(
                front_band,
                "-y",
                face_pos=(x_pos, z_pos),
                proud=proud_for_flush_mount(
                    knob,
                    axis="y",
                    asset_root=ASSETS.asset_root,
                ),
                asset_root=ASSETS.asset_root,
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=6.0),
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

    hood_shell = object_model.get_part("hood_shell")
    front_band = object_model.get_part("front_band")
    filter_panel = object_model.get_part("filter_panel")
    knobs = {
        "top": object_model.get_part("knob_top"),
        "left": object_model.get_part("knob_left"),
        "right": object_model.get_part("knob_right"),
        "bottom": object_model.get_part("knob_bottom"),
    }
    knob_joints = {
        name: object_model.get_articulation(f"front_band_to_knob_{name}")
        for name in knobs
    }

    ctx.expect_contact(front_band, hood_shell, name="front_band_connected_to_shell")
    ctx.expect_contact(filter_panel, hood_shell, name="filter_connected_to_shell")
    ctx.expect_within(
        filter_panel,
        hood_shell,
        axes="xy",
        name="filter_within_capture_body",
    )

    shell_aabb = ctx.part_world_aabb(hood_shell)
    band_aabb = ctx.part_world_aabb(front_band)
    if shell_aabb is not None and band_aabb is not None:
        shell_depth = shell_aabb[1][1] - shell_aabb[0][1]
        band_depth = band_aabb[1][1] - band_aabb[0][1]
        shell_width = shell_aabb[1][0] - shell_aabb[0][0]
        band_width = band_aabb[1][0] - band_aabb[0][0]
        ctx.check(
            "range_hood_proportions",
            band_depth < shell_depth and band_width > shell_width,
            (
                f"Expected a straight shallow front band ahead of a wider capture body; "
                f"band_depth={band_depth:.3f}, shell_depth={shell_depth:.3f}, "
                f"band_width={band_width:.3f}, shell_width={shell_width:.3f}"
            ),
        )

    non_fixed = [
        articulation
        for articulation in object_model.articulations
        if articulation.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "only_four_controls_articulate",
        len(non_fixed) == 4
        and {joint.name for joint in non_fixed}
        == {f"front_band_to_knob_{name}" for name in knobs},
        (
            "Expected exactly four movable articulations, one for each front knob; "
            f"found {[joint.name for joint in non_fixed]}."
        ),
    )

    for name, knob in knobs.items():
        joint = knob_joints[name]
        limits = joint.motion_limits
        ctx.expect_contact(knob, front_band, name=f"{name}_knob_contact_front_band")
        ctx.expect_overlap(
            knob,
            front_band,
            axes="xz",
            min_overlap=0.016,
            name=f"{name}_knob_centered_on_band",
        )
        ctx.check(
            f"{name}_knob_joint_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{joint.name} should be a continuous rotary control.",
        )
        ctx.check(
            f"{name}_knob_joint_axis_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis should be (0, 1, 0), got {joint.axis}.",
        )
        ctx.check(
            f"{name}_knob_joint_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            f"{joint.name} should use continuous limits without lower/upper bounds.",
        )

        with ctx.pose({joint: pi * 0.5}):
            ctx.expect_contact(knob, front_band, name=f"{name}_quarter_turn_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_quarter_turn_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{name}_quarter_turn_no_floating")
        with ctx.pose({joint: pi}):
            ctx.expect_contact(knob, front_band, name=f"{name}_half_turn_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_half_turn_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{name}_half_turn_no_floating")

    knob_positions = {name: ctx.part_world_position(knob) for name, knob in knobs.items()}
    if all(position is not None for position in knob_positions.values()):
        top = knob_positions["top"]
        left = knob_positions["left"]
        right = knob_positions["right"]
        bottom = knob_positions["bottom"]
        assert top is not None
        assert left is not None
        assert right is not None
        assert bottom is not None
        ctx.check(
            "diamond_knob_layout",
            (
                abs(top[0]) < 0.003
                and abs(bottom[0]) < 0.003
                and abs(left[2] - right[2]) < 0.002
                and abs(left[1] - right[1]) < 1e-6
                and abs(top[1] - left[1]) < 1e-6
                and left[0] < -0.01
                and right[0] > 0.01
                and top[2] > left[2] > bottom[2]
                and top[2] - bottom[2] < 0.04
                and right[0] - left[0] < 0.05
            ),
            (
                "Expected four tightly grouped knobs in a diamond pattern at the front center; "
                f"top={top}, left={left}, right={right}, bottom={bottom}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

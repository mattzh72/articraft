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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_windshield_sun_visor", assets=ASSETS)

    molded_black = model.material("molded_black", rgba=(0.12, 0.13, 0.14, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.42, 0.44, 0.47, 1.0))
    visor_shell_color = model.material("visor_shell_color", rgba=(0.26, 0.29, 0.25, 1.0))
    visor_pad_color = model.material("visor_pad_color", rgba=(0.33, 0.36, 0.32, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _padded_panel_mesh(
        *,
        width: float,
        height: float,
        thickness: float,
        bulge: float,
        radius: float,
    ):
        lower = [
            (x, y, -thickness * 0.5)
            for x, y in rounded_rect_profile(
                width * 0.985,
                height * 0.985,
                radius * 0.92,
                corner_segments=8,
            )
        ]
        middle = [
            (x, y, 0.0)
            for x, y in rounded_rect_profile(
                width,
                height,
                radius,
                corner_segments=8,
            )
        ]
        upper = [
            (x, y, thickness * 0.5 + bulge)
            for x, y in rounded_rect_profile(
                width * 0.985,
                height * 0.985,
                radius * 0.92,
                corner_segments=8,
            )
        ]
        return section_loft([lower, middle, upper])

    visor_shell_mesh = _save_mesh(
        "visor_shell.obj",
        _padded_panel_mesh(
            width=0.332,
            height=0.170,
            thickness=0.022,
            bulge=0.002,
            radius=0.022,
        ),
    )
    visor_pad_mesh = _save_mesh(
        "visor_face_pad.obj",
        _padded_panel_mesh(
            width=0.288,
            height=0.136,
            thickness=0.007,
            bulge=0.0015,
            radius=0.016,
        ),
    )

    roof_mount_base = model.part("roof_mount_base")
    roof_mount_base.visual(
        Box((0.118, 0.058, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, 0.005)),
        material=molded_black,
        name="roof_plate",
    )
    roof_mount_base.visual(
        Box((0.054, 0.036, 0.010)),
        origin=Origin(xyz=(0.004, 0.0, -0.005)),
        material=molded_black,
        name="reinforcement_block",
    )
    roof_mount_base.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=gunmetal,
        name="swivel_socket",
    )
    roof_mount_base.visual(
        Box((0.020, 0.006, 0.012)),
        origin=Origin(xyz=(0.008, 0.021, -0.013)),
        material=molded_black,
        name="left_swivel_rib",
    )
    roof_mount_base.visual(
        Box((0.020, 0.006, 0.012)),
        origin=Origin(xyz=(0.008, -0.021, -0.013)),
        material=molded_black,
        name="right_swivel_rib",
    )
    roof_mount_base.visual(
        Box((0.008, 0.024, 0.010)),
        origin=Origin(xyz=(-0.027, 0.0, -0.013)),
        material=molded_black,
        name="rear_swivel_stop",
    )
    for index, x_pos in enumerate((0.010, 0.064)):
        roof_mount_base.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(x_pos, 0.0, 0.012)),
            material=gunmetal,
            name=f"roof_fastener_{index}",
        )
    roof_mount_base.inertial = Inertial.from_geometry(
        Box((0.118, 0.058, 0.044)),
        mass=0.55,
        origin=Origin(xyz=(0.030, 0.0, -0.006)),
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=gunmetal,
        name="swivel_drum",
    )
    hinge_arm.visual(
        Box((0.018, 0.020, 0.010)),
        origin=Origin(xyz=(0.009, 0.0, -0.017)),
        material=painted_steel,
        name="arm_root_block",
    )
    hinge_arm.visual(
        Box((0.044, 0.018, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, -0.022)),
        material=painted_steel,
        name="arm_spine",
    )
    hinge_arm.visual(
        Box((0.030, 0.012, 0.006)),
        origin=Origin(xyz=(0.033, 0.0, -0.014)),
        material=painted_steel,
        name="arm_reinforcement_rib",
    )
    hinge_arm.visual(
        Box((0.020, 0.006, 0.022)),
        origin=Origin(xyz=(0.068, 0.012, -0.016)),
        material=painted_steel,
        name="left_clevis_ear",
    )
    hinge_arm.visual(
        Box((0.020, 0.006, 0.022)),
        origin=Origin(xyz=(0.068, -0.012, -0.016)),
        material=painted_steel,
        name="right_clevis_ear",
    )
    for index, y_pos in enumerate((0.017, -0.017)):
        hinge_arm.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(
                xyz=(0.068, y_pos, -0.016),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gunmetal,
            name=f"pivot_cap_{index}",
        )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.082, 0.040, 0.032)),
        mass=0.42,
        origin=Origin(xyz=(0.034, 0.0, -0.016)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gunmetal,
        name="hinge_boss",
    )
    visor_panel.visual(
        Box((0.040, 0.044, 0.022)),
        origin=Origin(xyz=(0.029, -0.012, 0.0)),
        material=painted_steel,
        name="mount_block",
    )
    visor_panel.visual(
        Box((0.070, 0.024, 0.010)),
        origin=Origin(xyz=(0.060, -0.012, 0.0)),
        material=painted_steel,
        name="mount_reinforcement",
    )
    visor_panel.visual(
        visor_shell_mesh,
        origin=Origin(xyz=(0.179, -0.082, 0.0)),
        material=visor_shell_color,
        name="visor_shell",
    )
    visor_panel.visual(
        visor_pad_mesh,
        origin=Origin(xyz=(0.191, -0.083, 0.008)),
        material=visor_pad_color,
        name="visor_face_pad",
    )
    for index, y_pos in enumerate((-0.026, 0.004)):
        visor_panel.visual(
            Cylinder(radius=0.004, length=0.006),
            origin=Origin(
                xyz=(0.050, y_pos, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=gunmetal,
            name=f"mount_fastener_{index}",
        )
    visor_panel.inertial = Inertial.from_geometry(
        Box((0.350, 0.176, 0.032)),
        mass=0.88,
        origin=Origin(xyz=(0.177, -0.082, 0.0)),
    )

    model.articulation(
        "base_to_hinge_arm",
        ArticulationType.REVOLUTE,
        parent=roof_mount_base,
        child=hinge_arm,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "hinge_arm_to_visor_panel",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.067, 0.0, -0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    roof_mount_base = object_model.get_part("roof_mount_base")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    base_to_hinge_arm = object_model.get_articulation("base_to_hinge_arm")
    hinge_arm_to_visor_panel = object_model.get_articulation("hinge_arm_to_visor_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected_parts_present",
        all(part is not None for part in (roof_mount_base, hinge_arm, visor_panel)),
        "Roof mount base, hinge arm, and visor panel must all exist.",
    )
    ctx.check(
        "joint_axes_are_mechanically_plausible",
        tuple(base_to_hinge_arm.axis) == (0.0, 0.0, 1.0)
        and tuple(hinge_arm_to_visor_panel.axis) == (1.0, 0.0, 0.0),
        "Secondary swing should be about vertical Z and visor drop should be about longitudinal X.",
    )

    ctx.expect_contact(
        roof_mount_base,
        hinge_arm,
        elem_a="swivel_socket",
        elem_b="swivel_drum",
        name="base_to_hinge_contact_rest",
    )
    ctx.expect_contact(
        hinge_arm,
        visor_panel,
        elem_a="left_clevis_ear",
        elem_b="hinge_boss",
        name="hinge_to_panel_contact_rest",
    )
    ctx.expect_gap(
        roof_mount_base,
        visor_panel,
        axis="z",
        positive_elem="roof_plate",
        negative_elem="visor_shell",
        min_gap=0.008,
        max_gap=0.030,
        name="visor_stows_below_roof_mount",
    )

    visor_rest_aabb = ctx.part_world_aabb(visor_panel)
    assert visor_rest_aabb is not None
    visor_rest_dims = (
        visor_rest_aabb[1][0] - visor_rest_aabb[0][0],
        visor_rest_aabb[1][1] - visor_rest_aabb[0][1],
        visor_rest_aabb[1][2] - visor_rest_aabb[0][2],
    )
    ctx.check(
        "visor_panel_realistic_size",
        0.32 <= visor_rest_dims[0] <= 0.38
        and 0.14 <= visor_rest_dims[1] <= 0.20
        and 0.020 <= visor_rest_dims[2] <= 0.040,
        f"Unexpected visor size {visor_rest_dims!r}.",
    )

    side_limits = base_to_hinge_arm.motion_limits
    assert side_limits is not None
    assert side_limits.upper is not None
    hinge_limits = hinge_arm_to_visor_panel.motion_limits
    assert hinge_limits is not None
    assert hinge_limits.upper is not None

    with ctx.pose({base_to_hinge_arm: side_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="side_swing_no_overlap")
        ctx.fail_if_isolated_parts(name="side_swing_no_floating")
        ctx.expect_contact(
            roof_mount_base,
            hinge_arm,
            elem_a="swivel_socket",
            elem_b="swivel_drum",
            name="base_to_hinge_contact_side_swing",
        )
        ctx.expect_contact(
            hinge_arm,
            visor_panel,
            elem_a="left_clevis_ear",
            elem_b="hinge_boss",
            name="hinge_to_panel_contact_side_swing",
        )
        visor_side_aabb = ctx.part_world_aabb(visor_panel)
        assert visor_side_aabb is not None
        ctx.check(
            "side_window_swing_moves_visor_outboard",
            visor_side_aabb[1][1] > visor_rest_aabb[1][1] + 0.12,
            f"Expected side swing to move visor toward +Y, got rest={visor_rest_aabb!r} swung={visor_side_aabb!r}.",
        )

    with ctx.pose({hinge_arm_to_visor_panel: hinge_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="visor_drop_no_overlap")
        ctx.fail_if_isolated_parts(name="visor_drop_no_floating")
        ctx.expect_contact(
            roof_mount_base,
            hinge_arm,
            elem_a="swivel_socket",
            elem_b="swivel_drum",
            name="base_to_hinge_contact_lowered",
        )
        ctx.expect_contact(
            hinge_arm,
            visor_panel,
            elem_a="left_clevis_ear",
            elem_b="hinge_boss",
            name="hinge_to_panel_contact_lowered",
        )
        visor_lowered_aabb = ctx.part_world_aabb(visor_panel)
        assert visor_lowered_aabb is not None
        ctx.check(
            "visor_rotates_down_from_roof",
            visor_lowered_aabb[0][2] < visor_rest_aabb[0][2] - 0.10,
            f"Expected lowered visor to extend downward, got rest={visor_rest_aabb!r} lowered={visor_lowered_aabb!r}.",
        )

    with ctx.pose(
        {
            base_to_hinge_arm: side_limits.upper,
            hinge_arm_to_visor_panel: math.radians(82.0),
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_side_and_drop_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_side_and_drop_no_floating")
        ctx.expect_contact(
            roof_mount_base,
            hinge_arm,
            elem_a="swivel_socket",
            elem_b="swivel_drum",
            name="base_to_hinge_contact_combined_pose",
        )
        ctx.expect_contact(
            hinge_arm,
            visor_panel,
            elem_a="left_clevis_ear",
            elem_b="hinge_boss",
            name="hinge_to_panel_contact_combined_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

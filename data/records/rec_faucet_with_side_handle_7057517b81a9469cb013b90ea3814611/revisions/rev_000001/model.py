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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_bench_cold_water_faucet")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.10, 0.10, 0.10, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.16, 0.43, 0.87, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    faucet_body = model.part("faucet_body")

    deck_plate_profile = rounded_rect_profile(0.118, 0.056, 0.022, corner_segments=12)
    deck_plate_mesh = _mesh(
        "faucet_deck_plate",
        ExtrudeGeometry.from_z0(deck_plate_profile, 0.006),
    )
    faucet_body.visual(deck_plate_mesh, material=brushed_steel, name="deck_plate")
    faucet_body.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brushed_steel,
        name="mount_shank",
    )
    faucet_body.visual(
        Cylinder(radius=0.021, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=gasket_black,
        name="body_gasket",
    )
    faucet_body.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=chrome,
        name="body_flange",
    )
    faucet_body.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=chrome,
        name="valve_body",
    )
    faucet_body.visual(
        Cylinder(radius=0.0215, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=chrome,
        name="bonnet",
    )
    faucet_body.visual(
        Cylinder(radius=0.0245, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=chrome,
        name="top_cap",
    )
    faucet_body.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, -0.004, 0.078)),
        material=chrome,
        name="spout_socket",
    )

    spout_neck = wire_from_points(
        [
            (0.0, -0.004, 0.052),
            (0.0, -0.004, 0.110),
            (0.0, 0.070, 0.110),
        ],
        radius=0.0105,
        radial_segments=20,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.022,
        corner_segments=12,
        up_hint=(1.0, 0.0, 0.0),
    )
    faucet_body.visual(
        _mesh("faucet_spout_neck", spout_neck),
        material=chrome,
        name="spout_neck",
    )
    faucet_body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.079, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="spout_nozzle",
    )
    faucet_body.visual(
        Cylinder(radius=0.0095, length=0.030),
        origin=Origin(xyz=(0.0, 0.088, 0.097)),
        material=chrome,
        name="outlet_tip",
    )
    faucet_body.visual(
        Cylinder(radius=0.0125, length=0.016),
        origin=Origin(xyz=(0.034, -0.004, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_boss",
    )
    faucet_body.inertial = Inertial.from_geometry(
        Box((0.14, 0.14, 0.18)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.026, 0.080)),
    )

    lever_handle = model.part("lever_handle")
    lever_handle.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_hub",
    )
    lever_handle.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_cap",
    )
    lever_arm = wire_from_points(
        [
            (0.029, 0.0, 0.0),
            (0.052, -0.002, 0.010),
            (0.079, -0.009, 0.030),
            (0.094, -0.013, 0.041),
        ],
        radius=0.0035,
        radial_segments=16,
        cap_ends=True,
        closed_path=False,
        corner_mode="fillet",
        corner_radius=0.010,
        corner_segments=10,
        up_hint=(0.0, 1.0, 0.0),
    )
    lever_handle.visual(
        _mesh("faucet_lever_arm", lever_arm),
        material=chrome,
        name="lever_arm",
    )
    lever_handle.visual(
        Sphere(radius=0.0065),
        origin=Origin(xyz=(0.094, -0.013, 0.041)),
        material=chrome,
        name="lever_tip",
    )
    lever_handle.visual(
        Sphere(radius=0.0032),
        origin=Origin(xyz=(0.093, -0.013, 0.041)),
        material=cold_blue,
        name="cold_index",
    )
    lever_handle.inertial = Inertial.from_geometry(
        Box((0.12, 0.04, 0.06)),
        mass=0.18,
        origin=Origin(xyz=(0.056, -0.006, 0.020)),
    )

    model.articulation(
        "lever_handle_joint",
        ArticulationType.REVOLUTE,
        parent=faucet_body,
        child=lever_handle,
        origin=Origin(xyz=(0.034, -0.004, 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    faucet_body = object_model.get_part("faucet_body")
    lever_handle = object_model.get_part("lever_handle")
    lever_joint = object_model.get_articulation("lever_handle_joint")

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
        lever_handle,
        faucet_body,
        elem_a="pivot_hub",
        elem_b="lever_boss",
        name="lever_pivot_seated_closed",
    )
    ctx.expect_origin_gap(
        lever_handle,
        faucet_body,
        axis="x",
        min_gap=0.025,
        max_gap=0.045,
        name="lever_is_side_mounted",
    )
    ctx.expect_origin_gap(
        lever_handle,
        faucet_body,
        axis="z",
        min_gap=0.040,
        max_gap=0.060,
        name="lever_is_mid_body_height",
    )
    ctx.expect_gap(
        faucet_body,
        faucet_body,
        axis="z",
        positive_elem="outlet_tip",
        negative_elem="deck_plate",
        min_gap=0.075,
        name="outlet_is_well_above_deck",
    )
    ctx.expect_gap(
        faucet_body,
        faucet_body,
        axis="y",
        positive_elem="outlet_tip",
        negative_elem="deck_plate",
        min_gap=0.045,
        name="spout_projects_forward_of_plate",
    )

    closed_tip = ctx.part_element_world_aabb(lever_handle, elem="lever_tip")
    assert closed_tip is not None

    with ctx.pose({lever_joint: 0.62}):
        ctx.expect_contact(
            lever_handle,
            faucet_body,
            elem_a="pivot_hub",
            elem_b="lever_boss",
            name="lever_pivot_seated_open",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_overlap_check")
        open_tip = ctx.part_element_world_aabb(lever_handle, elem="lever_tip")
        assert open_tip is not None
        assert open_tip[1][2] > closed_tip[1][2] + 0.025

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

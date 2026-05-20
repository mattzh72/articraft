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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_handle_kitchen_mixer_faucet")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.28, 0.29, 0.31, 1.0))

    faucet_body = model.part("faucet_body")

    deck_plate_geom = ExtrudeGeometry(
        rounded_rect_profile(0.22, 0.060, 0.018, corner_segments=10),
        0.008,
        center=True,
    )
    faucet_body.visual(
        _mesh("faucet_deck_plate", deck_plate_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=stainless,
        name="deck_plate",
    )

    pedestal_profile = [
        (0.0, 0.000),
        (0.031, 0.000),
        (0.034, 0.008),
        (0.034, 0.020),
        (0.031, 0.055),
        (0.037, 0.068),
        (0.031, 0.086),
        (0.029, 0.120),
        (0.028, 0.182),
        (0.024, 0.226),
        (0.021, 0.238),
        (0.0, 0.238),
    ]
    faucet_body.visual(
        _mesh("faucet_pedestal_shell", LatheGeometry(pedestal_profile, segments=56)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=stainless,
        name="pedestal_shell",
    )

    faucet_body.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.034, 0.108),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brushed_steel,
        name="handle_socket",
    )

    spout_path = [
        (0.0, 0.0, 0.224),
        (0.010, 0.0, 0.298),
        (0.060, 0.0, 0.378),
        (0.145, 0.0, 0.432),
        (0.220, 0.0, 0.392),
        (0.246, 0.0, 0.314),
    ]
    faucet_body.visual(
        _mesh(
            "faucet_gooseneck_spout",
            tube_from_spline_points(
                spout_path,
                radius=0.016,
                samples_per_segment=18,
                radial_segments=22,
                cap_ends=False,
            ),
        ),
        material=stainless,
        name="gooseneck_spout",
    )
    faucet_body.visual(
        Cylinder(radius=0.017, length=0.032),
        origin=Origin(xyz=(0.246, 0.0, 0.298)),
        material=stainless,
        name="nozzle_tip",
    )
    faucet_body.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.246, 0.0, 0.279)),
        material=dark_insert,
        name="aerator_ring",
    )

    handle_carrier = model.part("handle_carrier")
    handle_carrier.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.006, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brushed_steel,
        name="outer_spindle",
    )
    handle_carrier.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=brushed_steel,
        name="pivot_ball",
    )
    handle_carrier.visual(
        _mesh(
            "faucet_handle_knuckle_fairing",
            ExtrudeGeometry(
                rounded_rect_profile(0.020, 0.014, 0.0045, corner_segments=8),
                0.012,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_block",
    )
    handle_carrier.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.018, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="flow_barrel",
    )

    lever_handle = model.part("lever_handle")
    lever_handle.visual(
        Box((0.002, 0.006, 0.018)),
        origin=Origin(xyz=(0.007, 0.003, 0.009)),
        material=stainless,
        name="front_clevis",
    )
    lever_handle.visual(
        Box((0.002, 0.006, 0.018)),
        origin=Origin(xyz=(-0.007, 0.003, 0.009)),
        material=stainless,
        name="rear_clevis",
    )
    lever_handle.visual(
        Box((0.016, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.005, 0.016)),
        material=stainless,
        name="clevis_bridge",
    )
    lever_handle.visual(
        Box((0.008, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, 0.015)),
        material=stainless,
        name="stem_block",
    )
    lever_handle.visual(
        _mesh(
            "faucet_lever_base_shroud",
            ExtrudeGeometry(
                rounded_rect_profile(0.022, 0.014, 0.005, corner_segments=8),
                0.018,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.011, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="lever_base_shroud",
    )
    lever_handle.visual(
        _mesh(
            "faucet_lever_stem",
            tube_from_spline_points(
                [
                    (0.0, 0.011, 0.015),
                    (0.001, 0.025, 0.016),
                    (0.002, 0.050, 0.019),
                    (0.001, 0.066, 0.023),
                ],
                radius=0.0042,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=stainless,
        name="lever_stem",
    )
    lever_handle.visual(
        _mesh(
            "faucet_lever_grip",
            ExtrudeGeometry(
                rounded_rect_profile(0.024, 0.010, 0.0045, corner_segments=8),
                0.036,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.082, 0.024), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="lever_grip",
    )
    lever_handle.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.0995, 0.024)),
        material=stainless,
        name="lever_tip",
    )

    model.articulation(
        "body_to_handle_carrier",
        ArticulationType.REVOLUTE,
        parent=faucet_body,
        child=handle_carrier,
        origin=Origin(xyz=(0.0, 0.040, 0.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "carrier_to_lever_handle",
        ArticulationType.REVOLUTE,
        parent=handle_carrier,
        child=lever_handle,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    faucet_body = object_model.get_part("faucet_body")
    handle_carrier = object_model.get_part("handle_carrier")
    lever_handle = object_model.get_part("lever_handle")
    temp_joint = object_model.get_articulation("body_to_handle_carrier")
    flow_joint = object_model.get_articulation("carrier_to_lever_handle")

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

    body_aabb = ctx.part_world_aabb(faucet_body)
    body_height = None if body_aabb is None else body_aabb[1][2] - body_aabb[0][2]
    body_reach = None if body_aabb is None else body_aabb[1][0] - body_aabb[0][0]
    ctx.check(
        "faucet has tall high-arc proportions",
        body_height is not None
        and body_reach is not None
        and 0.40 <= body_height <= 0.48
        and body_reach >= 0.24,
        details=f"height={body_height}, reach={body_reach}",
    )

    nozzle_aabb = ctx.part_element_world_aabb(faucet_body, elem="nozzle_tip")
    deck_aabb = ctx.part_element_world_aabb(faucet_body, elem="deck_plate")
    ctx.check(
        "spout outlet sits well above the deck plate",
        nozzle_aabb is not None
        and deck_aabb is not None
        and nozzle_aabb[0][2] - deck_aabb[1][2] >= 0.26,
        details=f"nozzle={nozzle_aabb}, deck={deck_aabb}",
    )
    ctx.check(
        "spout projects forward from the pedestal",
        nozzle_aabb is not None and nozzle_aabb[1][0] >= 0.24,
        details=f"nozzle={nozzle_aabb}",
    )

    ctx.check(
        "temperature joint yaws about vertical axis",
        temp_joint.axis == (0.0, 0.0, 1.0)
        and temp_joint.motion_limits is not None
        and temp_joint.motion_limits.lower is not None
        and temp_joint.motion_limits.upper is not None
        and temp_joint.motion_limits.lower < 0.0 < temp_joint.motion_limits.upper,
        details=f"axis={temp_joint.axis}, limits={temp_joint.motion_limits}",
    )
    ctx.check(
        "flow joint lifts about a lateral axis",
        flow_joint.axis == (1.0, 0.0, 0.0)
        and flow_joint.motion_limits is not None
        and flow_joint.motion_limits.lower == 0.0
        and flow_joint.motion_limits.upper is not None
        and flow_joint.motion_limits.upper >= 0.80,
        details=f"axis={flow_joint.axis}, limits={flow_joint.motion_limits}",
    )

    ctx.expect_contact(
        handle_carrier,
        faucet_body,
        elem_a="outer_spindle",
        elem_b="handle_socket",
        name="handle carrier spindle seats in the body socket",
    )
    ctx.expect_contact(
        handle_carrier,
        lever_handle,
        elem_a="flow_barrel",
        elem_b="front_clevis",
        name="front clevis cheek contacts the flow barrel",
    )
    ctx.expect_contact(
        handle_carrier,
        lever_handle,
        elem_a="flow_barrel",
        elem_b="rear_clevis",
        name="rear clevis cheek contacts the flow barrel",
    )

    grip_rest = _aabb_center(ctx.part_element_world_aabb(lever_handle, elem="lever_grip"))
    with ctx.pose({flow_joint: 0.80}):
        grip_lifted = _aabb_center(ctx.part_element_world_aabb(lever_handle, elem="lever_grip"))
        ctx.expect_contact(
            handle_carrier,
            lever_handle,
            elem_a="flow_barrel",
            elem_b="front_clevis",
            name="front clevis stays seated while the lever lifts",
        )
        ctx.expect_contact(
            handle_carrier,
            lever_handle,
            elem_a="flow_barrel",
            elem_b="rear_clevis",
            name="rear clevis stays seated while the lever lifts",
        )
    ctx.check(
        "lever lifts upward to open flow",
        grip_rest is not None
        and grip_lifted is not None
        and grip_lifted[2] > grip_rest[2] + 0.030,
        details=f"rest={grip_rest}, lifted={grip_lifted}",
    )

    with ctx.pose({temp_joint: 0.45}):
        grip_yawed = _aabb_center(ctx.part_element_world_aabb(lever_handle, elem="lever_grip"))
    ctx.check(
        "side joystick pivots around the body waist for temperature",
        grip_rest is not None
        and grip_yawed is not None
        and abs(grip_yawed[0] - grip_rest[0]) > 0.020,
        details=f"rest={grip_rest}, yawed={grip_yawed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

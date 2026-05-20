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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_mixer_faucet", assets=ASSETS)

    chrome = model.material("chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.16, 0.34, 0.31)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.12, 0.155)),
    )

    body.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=chrome,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=chrome,
        name="lower_pedestal",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=chrome,
        name="mixer_core",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=chrome,
        name="top_cap",
    )
    body.visual(
        Box((0.010, 0.028, 0.040)),
        origin=Origin(xyz=(0.031, 0.0, 0.098)),
        material=chrome,
        name="handle_seat",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.015, 0.0, 0.098), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="cartridge_cap",
    )

    spout_shell = tube_from_spline_points(
        [
            (0.0, 0.0, 0.084),
            (0.0, 0.010, 0.132),
            (0.0, 0.055, 0.200),
            (0.0, 0.138, 0.258),
            (0.0, 0.225, 0.255),
            (0.0, 0.286, 0.201),
        ],
        radius=0.0175,
        samples_per_segment=18,
        radial_segments=24,
    )
    body.visual(_save_mesh("faucet_spout_shell.obj", spout_shell), material=chrome, name="spout_shell")
    body.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, 0.300, 0.201), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="nozzle_shell",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.008),
        origin=Origin(xyz=(0.0, 0.308, 0.201), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="aerator",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.13, 0.030, 0.040)),
        mass=0.24,
        origin=Origin(xyz=(0.060, 0.0, 0.014)),
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.054),
        origin=Origin(xyz=(0.020, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_arm",
    )
    handle.visual(
        _save_mesh(
            "faucet_lever_paddle.obj",
            ExtrudeGeometry(
                rounded_rect_profile(0.068, 0.022, 0.006, corner_segments=8),
                0.008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.075, 0.0, 0.018)),
        material=chrome,
        name="lever_tip",
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.050, 0.0, 0.098)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    handle_pivot = object_model.get_articulation("handle_pivot")

    deck_flange = body.get_visual("deck_flange")
    top_cap = body.get_visual("top_cap")
    handle_seat = body.get_visual("handle_seat")
    nozzle_shell = body.get_visual("nozzle_shell")
    hub = handle.get_visual("hub")
    lever_tip = handle.get_visual("lever_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    limits = handle_pivot.motion_limits
    ctx.check(
        "handle_joint_axis",
        handle_pivot.axis == (0.0, -1.0, 0.0),
        details=f"Expected side lever to pivot around -Y axis, got {handle_pivot.axis}.",
    )
    ctx.check(
        "handle_joint_limits_realistic",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and math.isclose(limits.lower, 0.0, abs_tol=1e-9)
        and 0.55 <= limits.upper <= 0.75,
        details=f"Expected lever lift range near 0 to 0.65 rad, got {limits}.",
    )

    ctx.expect_overlap(handle, body, axes="yz", min_overlap=0.018, elem_a=hub, elem_b=handle_seat)
    ctx.expect_gap(
        handle,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hub,
        negative_elem=handle_seat,
    )
    ctx.expect_contact(handle, body, elem_a=hub, elem_b=handle_seat)
    ctx.expect_gap(
        body,
        handle,
        axis="y",
        min_gap=0.20,
        positive_elem=nozzle_shell,
        negative_elem=lever_tip,
    )

    deck_aabb = ctx.part_element_world_aabb(body, elem=deck_flange)
    nozzle_aabb = ctx.part_element_world_aabb(body, elem=nozzle_shell)
    top_cap_aabb = ctx.part_element_world_aabb(body, elem=top_cap)
    rest_tip_aabb = ctx.part_element_world_aabb(handle, elem=lever_tip)

    if deck_aabb is None or nozzle_aabb is None or top_cap_aabb is None or rest_tip_aabb is None:
        ctx.fail("feature_aabbs_available", "Expected named faucet visuals to resolve to world AABBs.")
    else:
        ctx.check(
            "spout_reach_realistic",
            0.29 <= nozzle_aabb[1][1] <= 0.33,
            details=f"Nozzle max Y {nozzle_aabb[1][1]:.4f} m is outside a realistic kitchen faucet reach.",
        )
        ctx.check(
            "spout_height_realistic",
            0.18 <= nozzle_aabb[0][2] <= 0.23,
            details=f"Nozzle underside Z {nozzle_aabb[0][2]:.4f} m is outside a realistic spout height band.",
        )
        ctx.check(
            "handle_rest_above_deck",
            rest_tip_aabb[0][2] >= deck_aabb[1][2] + 0.055,
            details="Lever tip sits too low above the deck flange in the closed pose.",
        )
        ctx.check(
            "handle_rest_clear_of_top_cap",
            rest_tip_aabb[0][2] >= top_cap_aabb[1][2] - 0.002,
            details="Lever tip sags into the faucet cap in the closed pose.",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({handle_pivot: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_lower_no_floating")
            ctx.expect_contact(handle, body, elem_a=hub, elem_b=handle_seat, name="handle_lower_hub_contact")

        with ctx.pose({handle_pivot: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="handle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="handle_upper_no_floating")
            ctx.expect_contact(handle, body, elem_a=hub, elem_b=handle_seat, name="handle_upper_hub_contact")
            ctx.expect_gap(
                body,
                handle,
                axis="y",
                min_gap=0.20,
                positive_elem=nozzle_shell,
                negative_elem=lever_tip,
                name="handle_upper_nozzle_clearance",
            )
            upper_tip_aabb = ctx.part_element_world_aabb(handle, elem=lever_tip)
            if rest_tip_aabb is None or upper_tip_aabb is None:
                ctx.fail("handle_tip_pose_aabbs", "Expected lever tip AABBs in both rest and open poses.")
            else:
                rest_tip_center_z = 0.5 * (rest_tip_aabb[0][2] + rest_tip_aabb[1][2])
                upper_tip_center_z = 0.5 * (upper_tip_aabb[0][2] + upper_tip_aabb[1][2])
                ctx.check(
                    "handle_opens_upward",
                    upper_tip_center_z >= rest_tip_center_z + 0.035,
                    details=(
                        f"Lever tip center only lifted from {rest_tip_center_z:.4f} m "
                        f"to {upper_tip_center_z:.4f} m."
                    ),
                )

    with ctx.pose({handle_pivot: 0.32}):
        ctx.expect_gap(
            handle,
            body,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=hub,
            negative_elem=handle_seat,
            name="handle_mid_mount_seating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

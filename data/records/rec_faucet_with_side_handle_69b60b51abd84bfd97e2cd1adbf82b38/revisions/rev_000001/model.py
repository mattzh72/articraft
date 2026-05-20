from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _oval_plate_mesh(width: float, depth: float, thickness: float, name: str):
    profile = rounded_rect_profile(
        width,
        depth,
        radius=depth * 0.47,
        corner_segments=12,
    )
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(profile, thickness),
        name,
    )


def _lever_mesh(side: float, name: str):
    profile = rounded_rect_profile(0.016, 0.007, radius=0.0022, corner_segments=5)
    path = [
        (0.0, 0.0, 0.012),
        (side * 0.010, -0.004, 0.016),
        (side * 0.031, -0.016, 0.024),
        (side * 0.061, -0.031, 0.029),
    ]
    return mesh_from_geometry(
        sweep_profile_along_spline(
            path,
            profile=profile,
            samples_per_segment=18,
            cap_profile=True,
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laundry_utility_faucet")

    chrome = model.material("chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.43, 0.45, 0.48, 1.0))
    deck_finish = model.material("deck_finish", rgba=(0.93, 0.93, 0.91, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.36, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=deck_finish,
        name="deck_slab",
    )

    faucet_body = model.part("faucet_body")
    faucet_body.visual(
        _oval_plate_mesh(0.26, 0.072, 0.004, "base_plate_mesh"),
        material=chrome,
        name="base_plate",
    )
    faucet_body.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(-0.092, 0.0, 0.012)),
        material=chrome,
        name="hot_boss",
    )
    faucet_body.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.092, 0.0, 0.012)),
        material=chrome,
        name="cold_boss",
    )
    faucet_body.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chrome,
        name="spout_collar",
    )

    spout_path = [
        (0.0, 0.0, 0.004),
        (0.0, 0.0, 0.096),
        (0.0, 0.016, 0.108),
        (0.0, 0.052, 0.108),
        (0.0, 0.078, 0.108),
    ]
    faucet_body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                spout_path,
                radius=0.010,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
            "spout_neck_mesh",
        ),
        material=chrome,
        name="spout_neck",
    )
    faucet_body.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.0, 0.080, 0.108), rpy=(pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="spout_outlet",
    )
    faucet_body.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.088, 0.102)),
        material=dark_metal,
        name="aerator",
    )

    hot_handle = model.part("hot_handle")
    hot_handle.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=chrome,
        name="hot_cap",
    )
    hot_handle.visual(
        _lever_mesh(-1.0, "hot_lever_mesh"),
        material=chrome,
        name="hot_lever",
    )

    cold_handle = model.part("cold_handle")
    cold_handle.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=chrome,
        name="cold_cap",
    )
    cold_handle.visual(
        _lever_mesh(1.0, "cold_lever_mesh"),
        material=chrome,
        name="cold_lever",
    )

    model.articulation(
        "deck_to_faucet",
        ArticulationType.FIXED,
        parent=deck,
        child=faucet_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "faucet_to_hot_handle",
        ArticulationType.REVOLUTE,
        parent=faucet_body,
        child=hot_handle,
        origin=Origin(xyz=(-0.092, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "faucet_to_cold_handle",
        ArticulationType.REVOLUTE,
        parent=faucet_body,
        child=cold_handle,
        origin=Origin(xyz=(0.092, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    faucet_body = object_model.get_part("faucet_body")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")
    hot_joint = object_model.get_articulation("faucet_to_hot_handle")
    cold_joint = object_model.get_articulation("faucet_to_cold_handle")

    base_plate = faucet_body.get_visual("base_plate")
    hot_boss = faucet_body.get_visual("hot_boss")
    cold_boss = faucet_body.get_visual("cold_boss")
    hot_cap = hot_handle.get_visual("hot_cap")
    cold_cap = cold_handle.get_visual("cold_cap")
    hot_lever = hot_handle.get_visual("hot_lever")
    cold_lever = cold_handle.get_visual("cold_lever")

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
        faucet_body,
        deck,
        elem_a=base_plate,
        name="base plate sits on the deck",
    )
    ctx.expect_within(
        faucet_body,
        deck,
        axes="xy",
        inner_elem=base_plate,
        margin=0.002,
        name="oval base plate footprint stays on the deck",
    )
    ctx.expect_contact(
        hot_handle,
        faucet_body,
        elem_a=hot_cap,
        elem_b=hot_boss,
        name="hot handle cap seats on the left boss",
    )
    ctx.expect_contact(
        cold_handle,
        faucet_body,
        elem_a=cold_cap,
        elem_b=cold_boss,
        name="cold handle cap seats on the right boss",
    )

    hot_pos = ctx.part_world_position(hot_handle)
    cold_pos = ctx.part_world_position(cold_handle)
    ctx.check(
        "handles flank the centered spout",
        hot_pos is not None
        and cold_pos is not None
        and hot_pos[0] < -0.06
        and cold_pos[0] > 0.06,
        details=f"hot={hot_pos}, cold={cold_pos}",
    )

    hot_limits = hot_joint.motion_limits
    cold_limits = cold_joint.motion_limits
    ctx.check(
        "lever joints use realistic vertical valve rotation",
        hot_limits is not None
        and cold_limits is not None
        and abs(hot_joint.axis[0]) < 1e-9
        and abs(hot_joint.axis[1]) < 1e-9
        and abs(cold_joint.axis[0]) < 1e-9
        and abs(cold_joint.axis[1]) < 1e-9
        and hot_joint.axis[2] > 0.99
        and cold_joint.axis[2] > 0.99
        and isclose(hot_limits.lower or 0.0, 0.0, abs_tol=1e-9)
        and (hot_limits.upper or 0.0) >= 0.95
        and isclose(cold_limits.upper or 0.0, 0.0, abs_tol=1e-9)
        and (cold_limits.lower or 0.0) <= -0.95,
        details=(
            f"hot axis={hot_joint.axis}, hot limits={hot_limits}; "
            f"cold axis={cold_joint.axis}, cold limits={cold_limits}"
        ),
    )

    hot_closed_center = _aabb_center(ctx.part_element_world_aabb(hot_handle, elem=hot_lever))
    cold_closed_center = _aabb_center(ctx.part_element_world_aabb(cold_handle, elem=cold_lever))
    with ctx.pose({hot_joint: hot_limits.upper, cold_joint: cold_limits.lower}):
        hot_open_center = _aabb_center(ctx.part_element_world_aabb(hot_handle, elem=hot_lever))
        cold_open_center = _aabb_center(ctx.part_element_world_aabb(cold_handle, elem=cold_lever))
        ctx.check(
            "handles sweep inward and rearward as they open",
            hot_closed_center is not None
            and cold_closed_center is not None
            and hot_open_center is not None
            and cold_open_center is not None
            and hot_open_center[1] < hot_closed_center[1] - 0.015
            and cold_open_center[1] < cold_closed_center[1] - 0.015
            and abs(hot_open_center[0]) < abs(hot_closed_center[0]) - 0.015
            and abs(cold_open_center[0]) < abs(cold_closed_center[0]) - 0.015,
            details=(
                f"hot closed={hot_closed_center}, hot open={hot_open_center}, "
                f"cold closed={cold_closed_center}, cold open={cold_open_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

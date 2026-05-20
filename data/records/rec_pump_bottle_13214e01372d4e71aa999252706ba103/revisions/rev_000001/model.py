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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_bottle_shell():
    outer_profile = [
        (0.0, 0.000),
        (0.033, 0.000),
        (0.036, 0.010),
        (0.036, 0.138),
        (0.034, 0.152),
        (0.028, 0.164),
        (0.021, 0.174),
        (0.017, 0.182),
        (0.017, 0.188),
    ]
    inner_profile = [
        (0.0, 0.003),
        (0.0305, 0.003),
        (0.0330, 0.012),
        (0.0330, 0.137),
        (0.0310, 0.151),
        (0.0255, 0.163),
        (0.0185, 0.173),
        (0.0135, 0.182),
        (0.0135, 0.186),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=64),
        "pump_bottle_shell",
    )


def _build_collar_shell():
    outer_profile = [
        (0.0185, 0.000),
        (0.0210, 0.004),
        (0.0210, 0.018),
        (0.0190, 0.022),
    ]
    inner_profile = [
        (0.0105, 0.000),
        (0.0105, 0.018),
        (0.0100, 0.022),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=48),
        "pump_collar_shell",
    )


def _build_dust_cap_shell():
    outer_profile = [
        (0.0058, 0.000),
        (0.0058, 0.014),
        (0.0048, 0.017),
        (0.0034, 0.0185),
    ]
    inner_profile = [
        (0.0046, 0.0010),
        (0.0046, 0.0132),
        (0.0034, 0.0170),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=40),
        "pump_dust_cap_shell_v4",
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pump_bottle")

    bottle_white = model.material("bottle_white", rgba=(0.96, 0.95, 0.93, 1.0))
    pump_black = model.material("pump_black", rgba=(0.16, 0.17, 0.18, 1.0))
    nozzle_white = model.material("nozzle_white", rgba=(0.94, 0.94, 0.95, 1.0))
    cap_translucent = model.material("cap_translucent", rgba=(0.72, 0.77, 0.82, 0.55))

    bottle = model.part("bottle")
    bottle.visual(_build_bottle_shell(), material=bottle_white, name="bottle_shell")
    bottle.visual(
        Cylinder(radius=0.0145, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=bottle_white,
        name="neck_core",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.072, 0.072, 0.188)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
    )

    collar = model.part("collar")
    collar.visual(
        Box((0.030, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, 0.0135, 0.011)),
        material=pump_black,
        name="collar_front_wall",
    )
    collar.visual(
        Box((0.030, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, -0.0135, 0.011)),
        material=pump_black,
        name="collar_back_wall",
    )
    collar.visual(
        Box((0.003, 0.024, 0.022)),
        origin=Origin(xyz=(-0.0135, 0.0, 0.011)),
        material=pump_black,
        name="collar_left_wall",
    )
    collar.visual(
        Box((0.006, 0.024, 0.022)),
        origin=Origin(xyz=(0.015, 0.0, 0.011)),
        material=pump_black,
        name="collar_shell",
    )
    collar.visual(
        Box((0.014, 0.008, 0.014)),
        origin=Origin(xyz=(0.006, 0.006, 0.028)),
        material=pump_black,
        name="hinge_bridge",
    )
    collar.visual(
        Box((0.004, 0.008, 0.006)),
        origin=Origin(xyz=(0.006, 0.006, 0.035)),
        material=pump_black,
        name="hinge_block",
    )
    collar.inertial = Inertial.from_geometry(
        Box((0.036, 0.032, 0.035)),
        mass=0.08,
        origin=Origin(xyz=(0.003, 0.0, 0.0175)),
    )

    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=nozzle_white,
        name="guide",
    )
    plunger.visual(
        Cylinder(radius=0.0040, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=nozzle_white,
        name="stem",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0045, length=0.028),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
    )

    model.articulation(
        "collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.06,
            lower=0.0,
            upper=0.008,
        ),
    )

    nozzle = model.part("nozzle_head")
    nozzle.visual(
        Cylinder(radius=0.0042, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=nozzle_white,
        name="hub",
    )
    nozzle.visual(
        Box((0.016, 0.012, 0.008)),
        origin=Origin(xyz=(0.006, 0.0, 0.006)),
        material=nozzle_white,
        name="neck_bridge",
    )
    nozzle.visual(
        Box((0.024, 0.018, 0.012)),
        origin=Origin(xyz=(0.016, 0.0, 0.008)),
        material=nozzle_white,
        name="head",
    )
    nozzle.visual(
        Cylinder(radius=0.0038, length=0.022),
        origin=Origin(xyz=(0.029, 0.0, 0.009), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_white,
        name="spout",
    )
    nozzle.visual(
        Cylinder(radius=0.0028, length=0.008),
        origin=Origin(xyz=(0.041, 0.0, 0.009), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_white,
        name="spout_tip",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.050, 0.022, 0.022)),
        mass=0.06,
        origin=Origin(xyz=(0.020, 0.0, 0.004)),
    )

    model.articulation(
        "plunger_to_nozzle_head",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-0.9,
            upper=0.9,
        ),
    )

    dust_cap = model.part("dust_cap")
    dust_cap.visual(
        Box((0.004, 0.008, 0.006)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=cap_translucent,
        name="hinge_leaf",
    )
    dust_cap.visual(
        Box((0.004, 0.004, 0.018)),
        origin=Origin(xyz=(0.004, 0.0, 0.009)),
        material=cap_translucent,
        name="cap_riser",
    )
    dust_cap.visual(
        Box((0.028, 0.004, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.016)),
        material=cap_translucent,
        name="cap_arm",
    )
    dust_cap.visual(
        Box((0.004, 0.004, 0.012)),
        origin=Origin(xyz=(0.032, 0.0, 0.010)),
        material=cap_translucent,
        name="cap_drop",
    )
    dust_cap.visual(
        Box((0.002, 0.004, 0.0012)),
        origin=Origin(xyz=(0.035, 0.0, 0.0164)),
        material=cap_translucent,
        name="cap_shell_bridge",
    )
    dust_cap.visual(
        _build_dust_cap_shell(),
        origin=Origin(xyz=(0.036, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_translucent,
        name="cap_shell",
    )
    dust_cap.inertial = Inertial.from_geometry(
        Box((0.050, 0.012, 0.026)),
        mass=0.025,
        origin=Origin(xyz=(0.020, 0.0, 0.013)),
    )

    model.articulation(
        "collar_to_dust_cap",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=dust_cap,
        origin=Origin(xyz=(0.008, 0.006, 0.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    plunger = object_model.get_part("plunger")
    nozzle = object_model.get_part("nozzle_head")
    dust_cap = object_model.get_part("dust_cap")

    plunger_slide = object_model.get_articulation("collar_to_plunger")
    nozzle_swivel = object_model.get_articulation("plunger_to_nozzle_head")
    cap_hinge = object_model.get_articulation("collar_to_dust_cap")

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

    ctx.check(
        "plunger_slide_axis",
        tuple(plunger_slide.axis) == (0.0, 0.0, -1.0),
        details=f"expected axial downward slide, got axis={plunger_slide.axis}",
    )
    ctx.check(
        "nozzle_swivel_axis",
        tuple(nozzle_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical stem-axis rotation, got axis={nozzle_swivel.axis}",
    )
    ctx.check(
        "dust_cap_hinge_axis",
        tuple(cap_hinge.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"expected lateral cap hinge about local y, got axis={cap_hinge.axis}",
    )

    ctx.expect_contact(collar, bottle)
    ctx.expect_contact(nozzle, plunger, elem_a="hub", elem_b="stem")
    ctx.expect_contact(dust_cap, collar, elem_a="hinge_leaf", elem_b="hinge_block")
    ctx.expect_overlap(dust_cap, nozzle, axes="yz", min_overlap=0.002, elem_a="cap_shell", elem_b="spout_tip")

    bottle_aabb = ctx.part_world_aabb(bottle)
    assert bottle_aabb is not None
    bottle_height = bottle_aabb[1][2] - bottle_aabb[0][2]
    ctx.check(
        "bottle_realistic_height",
        0.18 <= bottle_height <= 0.20,
        details=f"bottle height {bottle_height:.4f} m is outside realistic range",
    )

    plunger_rest = ctx.part_world_position(plunger)
    assert plunger_rest is not None
    with ctx.pose({plunger_slide: 0.008}):
        plunger_pressed = ctx.part_world_position(plunger)
        assert plunger_pressed is not None
        ctx.check(
            "plunger_descends",
            plunger_pressed[2] < plunger_rest[2] - 0.0075,
            details=(
                f"expected at least 7.5 mm downward travel, "
                f"rest_z={plunger_rest[2]:.4f}, pressed_z={plunger_pressed[2]:.4f}"
            ),
        )

    spout_rest = ctx.part_element_world_aabb(nozzle, elem="spout")
    assert spout_rest is not None
    spout_rest_center = _aabb_center(spout_rest)
    with ctx.pose({cap_hinge: 1.8, nozzle_swivel: 0.8}):
        spout_turned = ctx.part_element_world_aabb(nozzle, elem="spout")
        assert spout_turned is not None
        spout_turned_center = _aabb_center(spout_turned)
        ctx.check(
            "nozzle_rotates_about_stem",
            (
                spout_turned_center[1] > spout_rest_center[1] + 0.014
                and abs(spout_turned_center[2] - spout_rest_center[2]) <= 0.002
            ),
            details=(
                "expected spout to swing laterally with little height change; "
                f"rest_center={spout_rest_center}, turned_center={spout_turned_center}"
            ),
        )

    cap_rest = ctx.part_world_aabb(dust_cap)
    assert cap_rest is not None
    cap_shell_rest = ctx.part_element_world_aabb(dust_cap, elem="cap_shell")
    assert cap_shell_rest is not None
    cap_shell_rest_center = _aabb_center(cap_shell_rest)
    with ctx.pose({cap_hinge: 1.8}):
        cap_open = ctx.part_world_aabb(dust_cap)
        cap_shell_open = ctx.part_element_world_aabb(dust_cap, elem="cap_shell")
        assert cap_open is not None
        assert cap_shell_open is not None
        cap_shell_open_center = _aabb_center(cap_shell_open)
        ctx.check(
            "dust_cap_flips_clear",
            (
                cap_shell_open_center[2] > cap_shell_rest_center[2] + 0.018
                and cap_shell_open_center[0] < cap_shell_rest_center[0] - 0.030
            ),
            details=(
                "expected cap shell to swing backward and upward; "
                f"rest_center={cap_shell_rest_center}, open_center={cap_shell_open_center}, "
                f"open_aabb={cap_open}"
            ),
        )
        ctx.expect_gap(dust_cap, nozzle, axis="z", min_gap=0.012, positive_elem="cap_shell")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _merge_geometries(*geometries):
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _build_bottle_shell_mesh():
    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.031, 0.000),
            (0.039, 0.008),
            (0.042, 0.020),
            (0.042, 0.158),
            (0.041, 0.174),
            (0.037, 0.190),
            (0.031, 0.205),
            (0.027, 0.216),
            (0.0235, 0.226),
            (0.0220, 0.235),
        ],
        [
            (0.000, 0.0035),
            (0.0375, 0.010),
            (0.0390, 0.022),
            (0.0390, 0.156),
            (0.0380, 0.172),
            (0.0345, 0.188),
            (0.0290, 0.203),
            (0.0255, 0.214),
            (0.0210, 0.226),
            (0.0130, 0.235),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    foot_ring = CylinderGeometry(radius=0.0325, height=0.003, radial_segments=48).translate(
        0.0,
        0.0,
        0.0015,
    )
    return _merge_geometries(bottle_shell, foot_ring)


def _build_upper_housing_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.0210, 0.000),
            (0.0230, 0.004),
            (0.0240, 0.010),
            (0.0235, 0.018),
            (0.0225, 0.026),
            (0.0215, 0.034),
        ],
        [
            (0.0140, 0.000),
            (0.0145, 0.006),
            (0.0145, 0.016),
            (0.0140, 0.026),
            (0.0135, 0.034),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    spout_bridge = BoxGeometry((0.016, 0.012, 0.010)).translate(0.014, 0.0, 0.021)
    spout_pedestal = CylinderGeometry(
        radius=0.0065,
        height=0.010,
        radial_segments=24,
    ).translate(0.0215, 0.0, 0.029)
    return _merge_geometries(shell, spout_bridge, spout_pedestal)


def _build_plunger_mesh():
    shaft = CylinderGeometry(radius=0.0070, height=0.065, radial_segments=30).translate(
        0.0,
        0.0,
        -0.018,
    )
    button = CylinderGeometry(radius=0.0110, height=0.010, radial_segments=30).translate(
        0.0,
        0.0,
        0.005,
    )
    cap = DomeGeometry(radius=0.0110, radial_segments=30, height_segments=14, closed=True).translate(
        0.0,
        0.0,
        0.010,
    )
    return _merge_geometries(shaft, button, cap)


def _build_spout_mesh():
    hub = CylinderGeometry(radius=0.0075, height=0.009, radial_segments=24).translate(
        0.0,
        0.0,
        0.0045,
    )
    spout_tube = tube_from_spline_points(
        [
            (0.000, 0.000, 0.0045),
            (0.016, 0.000, 0.0090),
            (0.048, 0.000, 0.0130),
            (0.090, 0.000, 0.0110),
            (0.122, 0.000, 0.0030),
            (0.140, 0.000, -0.0060),
        ],
        radius=0.0040,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    nozzle_tip = CylinderGeometry(radius=0.0025, height=0.016, radial_segments=18)
    nozzle_tip.rotate_y(math.pi / 2.0)
    nozzle_tip.translate(0.146, 0.0, -0.0060)
    return _merge_geometries(hub, spout_tube, nozzle_tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="salon_pump_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.90, 0.88, 0.82, 1.0))
    pump_black = model.material("pump_black", rgba=(0.10, 0.10, 0.11, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        mesh_from_geometry(_build_bottle_shell_mesh(), "bottle_shell"),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.235),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
    )

    upper_housing = model.part("upper_housing")
    upper_housing.visual(
        mesh_from_geometry(_build_upper_housing_mesh(), "upper_housing_shell"),
        material=pump_black,
        name="upper_housing_shell",
    )
    upper_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.034),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        mesh_from_geometry(_build_plunger_mesh(), "plunger_mesh"),
        material=pump_black,
        name="plunger_mesh",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.080),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    spout = model.part("spout")
    spout.visual(
        mesh_from_geometry(_build_spout_mesh(), "dispensing_spout"),
        material=pump_black,
        name="dispensing_spout",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.160, 0.020, 0.024)),
        mass=0.035,
        origin=Origin(xyz=(0.076, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_upper_housing",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=upper_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
    )
    model.articulation(
        "upper_housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=upper_housing,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.10,
            lower=0.0,
            upper=0.012,
        ),
    )
    model.articulation(
        "upper_housing_to_spout",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=spout,
        origin=Origin(xyz=(0.0215, 0.0, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-1.4,
            upper=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    bottle_body = object_model.get_part("bottle_body")
    upper_housing = object_model.get_part("upper_housing")
    plunger = object_model.get_part("plunger")
    spout = object_model.get_part("spout")

    plunger_joint = object_model.get_articulation("upper_housing_to_plunger")
    spout_joint = object_model.get_articulation("upper_housing_to_spout")

    ctx.check(
        "plunger translates along the bottle axis",
        tuple(plunger_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={plunger_joint.axis}",
    )
    ctx.check(
        "spout swivels around a vertical pivot",
        tuple(spout_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spout_joint.axis}",
    )

    with ctx.pose({plunger_joint: 0.0, spout_joint: 0.0}):
        ctx.expect_gap(
            upper_housing,
            bottle_body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="upper housing seats on the bottle neck",
        )
        ctx.expect_overlap(
            upper_housing,
            bottle_body,
            axes="xy",
            min_overlap=0.030,
            name="upper housing stays centered over the bottle finish",
        )
        bottle_aabb = ctx.part_world_aabb(bottle_body)
        spout_aabb = ctx.part_world_aabb(spout)
        ctx.check(
            "spout projects clearly beyond the bottle shoulder",
            bottle_aabb is not None
            and spout_aabb is not None
            and spout_aabb[1][0] > bottle_aabb[1][0] + 0.070,
            details=f"bottle_aabb={bottle_aabb}, spout_aabb={spout_aabb}",
        )

        rest_plunger_aabb = ctx.part_world_aabb(plunger)
        rest_spout_aabb = spout_aabb

    pressed_travel = plunger_joint.motion_limits.upper if plunger_joint.motion_limits else None
    with ctx.pose({plunger_joint: pressed_travel if pressed_travel is not None else 0.0}):
        pressed_plunger_aabb = ctx.part_world_aabb(plunger)
        ctx.expect_overlap(
            plunger,
            upper_housing,
            axes="xy",
            min_overlap=0.020,
            name="pressed plunger remains guided inside the head",
        )

    ctx.check(
        "plunger depresses downward into the head",
        rest_plunger_aabb is not None
        and pressed_plunger_aabb is not None
        and pressed_plunger_aabb[1][2] < rest_plunger_aabb[1][2] - 0.010,
        details=f"rest_plunger_aabb={rest_plunger_aabb}, pressed_plunger_aabb={pressed_plunger_aabb}",
    )

    swivel_angle = 1.20
    with ctx.pose({spout_joint: swivel_angle}):
        turned_spout_aabb = ctx.part_world_aabb(spout)

    if rest_spout_aabb is not None and turned_spout_aabb is not None:
        rest_center_y = 0.5 * (rest_spout_aabb[0][1] + rest_spout_aabb[1][1])
        turned_center_y = 0.5 * (turned_spout_aabb[0][1] + turned_spout_aabb[1][1])
        rest_center_z = 0.5 * (rest_spout_aabb[0][2] + rest_spout_aabb[1][2])
        turned_center_z = 0.5 * (turned_spout_aabb[0][2] + turned_spout_aabb[1][2])
    else:
        rest_center_y = None
        turned_center_y = None
        rest_center_z = None
        turned_center_z = None

    ctx.check(
        "spout can swing sideways without drooping off the head",
        rest_center_y is not None
        and turned_center_y is not None
        and rest_center_z is not None
        and turned_center_z is not None
        and turned_center_y > rest_center_y + 0.050
        and abs(turned_center_z - rest_center_z) < 0.010,
        details=(
            f"rest_center_y={rest_center_y}, turned_center_y={turned_center_y}, "
            f"rest_center_z={rest_center_z}, turned_center_z={turned_center_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

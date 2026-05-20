from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BARREL_CENTER_Y = 0.02
BARREL_RADIUS = 0.041
BARREL_LENGTH = 0.62
BARREL_TOP_Z = 0.675
HANDLE_TRAVEL = 0.26


def _ring_mesh(name: str, *, outer_radius: float, inner_radius: float, front_z: float, rear_z: float):
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, front_z), (outer_radius, rear_z)],
        [(inner_radius, front_z), (inner_radius, rear_z)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(-pi / 2.0)
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_track_pump")

    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    handle_plastic = model.material("handle_plastic", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.30, 0.12, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_steel,
        name="base_beam",
    )
    body.visual(
        Box((0.11, 0.22, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_steel,
        name="base_spine",
    )
    body.visual(
        Box((0.10, 0.11, 0.006)),
        origin=Origin(xyz=(-0.090, -0.010, 0.025)),
        material=black_rubber,
        name="left_tread",
    )
    body.visual(
        Box((0.10, 0.11, 0.006)),
        origin=Origin(xyz=(0.090, -0.010, 0.025)),
        material=black_rubber,
        name="right_tread",
    )
    body.visual(
        Cylinder(radius=0.053, length=0.070),
        origin=Origin(xyz=(0.0, BARREL_CENTER_Y, 0.055)),
        material=dark_steel,
        name="lower_collar",
    )
    body.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(xyz=(0.0, BARREL_CENTER_Y, 0.340)),
        material=steel,
        name="barrel",
    )
    body.visual(
        Cylinder(radius=0.049, length=0.050),
        origin=Origin(xyz=(0.0, BARREL_CENTER_Y, 0.650)),
        material=dark_steel,
        name="top_collar",
    )
    body.visual(
        Box((0.014, 0.010, 0.028)),
        origin=Origin(xyz=(-0.017, BARREL_CENTER_Y, BARREL_TOP_Z)),
        material=dark_steel,
        name="gland_left",
    )
    body.visual(
        Box((0.014, 0.010, 0.028)),
        origin=Origin(xyz=(0.017, BARREL_CENTER_Y, BARREL_TOP_Z)),
        material=dark_steel,
        name="gland_right",
    )
    body.visual(
        Box((0.020, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, BARREL_CENTER_Y - 0.017, BARREL_TOP_Z)),
        material=dark_steel,
        name="gland_front",
    )
    body.visual(
        Box((0.020, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, BARREL_CENTER_Y + 0.017, BARREL_TOP_Z)),
        material=dark_steel,
        name="gland_rear",
    )
    body.visual(
        Box((0.028, 0.074, 0.022)),
        origin=Origin(xyz=(0.0, -0.062, 0.094)),
        material=dark_steel,
        name="gauge_pedestal",
    )
    body.visual(
        Box((0.026, 0.020, 0.060)),
        origin=Origin(xyz=(0.047, 0.044, 0.470)),
        material=dark_steel,
        name="hose_brace",
    )
    body.visual(
        Box((0.014, 0.016, 0.040)),
        origin=Origin(xyz=(0.060, 0.056, 0.470)),
        material=dark_steel,
        name="hose_dock",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.009, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=steel,
        name="rod",
    )
    handle.visual(
        Box((0.034, 0.034, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=dark_steel,
        name="carrier",
    )
    handle.visual(
        Box((0.28, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=handle_plastic,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.019, length=0.085),
        origin=Origin(xyz=(-0.097, 0.0, 0.162), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="left_grip",
    )
    handle.visual(
        Cylinder(radius=0.019, length=0.085),
        origin=Origin(xyz=(0.097, 0.0, 0.162), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_rubber,
        name="right_grip",
    )

    model.articulation(
        "pump_stroke",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, BARREL_CENTER_Y, BARREL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.70,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )

    gauge_white = model.material("gauge_white", rgba=(0.93, 0.94, 0.92, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.78, 0.86, 0.92, 0.28))
    alert_red = model.material("alert_red", rgba=(0.72, 0.14, 0.12, 1.0))
    hose_black = model.material("hose_black", rgba=(0.07, 0.07, 0.08, 1.0))

    gauge = model.part("gauge")
    gauge.visual(
        _ring_mesh(
            "gauge_housing",
            outer_radius=0.053,
            inner_radius=0.041,
            front_z=-0.010,
            rear_z=0.008,
        ),
        material=dark_steel,
        name="housing",
    )
    gauge.visual(
        Cylinder(radius=0.041, length=0.001),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=gauge_white,
        name="dial_face",
    )
    gauge.visual(
        Cylinder(radius=0.041, length=0.001),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=lens_clear,
        name="lens",
    )
    gauge.visual(
        Cylinder(radius=0.041, length=0.002),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_cap",
    )
    gauge.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="stem",
    )
    gauge.visual(
        Cylinder(radius=0.0035, length=0.001),
        origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spindle",
    )
    gauge.visual(
        Box((0.026, 0.012, 0.014)),
        origin=Origin(xyz=(0.034, -0.001, 0.034)),
        material=dark_steel,
        name="button_mount",
    )

    needle = model.part("needle")
    needle.visual(
        Box((0.030, 0.0012, 0.003)),
        origin=Origin(xyz=(0.015, -0.002, 0.0)),
        material=alert_red,
        name="pointer",
    )
    needle.visual(
        Box((0.010, 0.0012, 0.003)),
        origin=Origin(xyz=(-0.005, -0.002, 0.0)),
        material=alert_red,
        name="counterweight",
    )
    needle.visual(
        Cylinder(radius=0.0045, length=0.002),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot",
    )

    button = model.part("button")
    button.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(0.0, 0.0, 0.0)),
        material=alert_red,
        name="button_cap",
    )

    hose = model.part("hose")
    hose.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.033, -0.078, 0.104),
                    (0.068, -0.060, 0.092),
                    (0.085, -0.010, 0.118),
                    (0.082, 0.044, 0.258),
                    (0.075, 0.056, 0.430),
                ],
                radius=0.007,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "pump_hose",
        ),
        material=hose_black,
        name="hose_tube",
    )
    hose.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(0.075, 0.056, 0.480)),
        material=steel,
        name="nozzle",
    )

    model.articulation(
        "gauge_mount",
        ArticulationType.FIXED,
        parent=body,
        child=gauge,
        origin=Origin(xyz=(0.0, -0.107, 0.105)),
    )
    model.articulation(
        "needle_sweep",
        ArticulationType.REVOLUTE,
        parent=gauge,
        child=needle,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.05,
            velocity=4.0,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "button_press",
        ArticulationType.PRISMATIC,
        parent=gauge,
        child=button,
        origin=Origin(xyz=(0.041, -0.001, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.04,
            lower=0.0,
            upper=0.002,
        ),
    )
    model.articulation(
        "hose_mount",
        ArticulationType.FIXED,
        parent=body,
        child=hose,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    gauge = object_model.get_part("gauge")
    needle = object_model.get_part("needle")
    button = object_model.get_part("button")
    stroke = object_model.get_articulation("pump_stroke")
    needle_sweep = object_model.get_articulation("needle_sweep")
    button_press = object_model.get_articulation("button_press")
    limits = stroke.motion_limits
    needle_limits = needle_sweep.motion_limits
    button_limits = button_press.motion_limits

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({stroke: limits.lower}):
            ctx.expect_overlap(
                handle,
                body,
                axes="xy",
                elem_a="rod",
                elem_b="barrel",
                min_overlap=0.016,
                name="rod footprint stays centered over barrel at rest",
            )
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel",
                margin=0.0,
                name="rod stays within barrel footprint at rest",
            )
            rest_pos = ctx.part_world_position(handle)

        with ctx.pose({stroke: limits.upper}):
            ctx.expect_overlap(
                handle,
                body,
                axes="xy",
                elem_a="rod",
                elem_b="barrel",
                min_overlap=0.016,
                name="rod footprint stays centered over barrel when lifted",
            )
            ctx.expect_within(
                handle,
                body,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel",
                margin=0.0,
                name="rod stays within barrel footprint when lifted",
            )
            extended_pos = ctx.part_world_position(handle)

        ctx.check(
            "handle lifts upward along barrel axis",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    ctx.expect_within(
        needle,
        gauge,
        axes="xz",
        inner_elem="pointer",
        outer_elem="dial_face",
        margin=0.012,
        name="needle remains inside gauge face",
    )

    if needle_limits is not None and needle_limits.upper is not None:
        rest_pointer = ctx.part_element_world_aabb(needle, elem="pointer")
        with ctx.pose({needle_sweep: needle_limits.upper}):
            swept_pointer = ctx.part_element_world_aabb(needle, elem="pointer")
        ctx.check(
            "needle rotates upward across the dial",
            rest_pointer is not None
            and swept_pointer is not None
            and swept_pointer[1][2] > rest_pointer[1][2] + 0.012,
            details=f"rest={rest_pointer}, swept={swept_pointer}",
        )

    if button_limits is not None and button_limits.upper is not None:
        rest_button = ctx.part_world_position(button)
        with ctx.pose({button_press: button_limits.upper}):
            pressed_button = ctx.part_world_position(button)
        ctx.check(
            "pressure release button translates on its guide",
            rest_button is not None
            and pressed_button is not None
            and pressed_button[2] > rest_button[2] + 0.0015,
            details=f"rest={rest_button}, pressed={pressed_button}",
        )

    return ctx.report()


object_model = build_object_model()

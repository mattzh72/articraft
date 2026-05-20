from __future__ import annotations

from math import atan2, pi, sqrt

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


ROTOR_CENTERS = (
    (0.37, 0.27),
    (0.37, -0.27),
    (-0.37, 0.27),
    (-0.37, -0.27),
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    """CadQuery rounded-vertical-corner box, centered on its local origin."""
    shape = cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_quadcopter_drone")

    matte_black = _material("matte_black", (0.015, 0.016, 0.018, 1.0))
    carbon = _material("satin_carbon", (0.035, 0.038, 0.042, 1.0))
    dark_gray = _material("dark_graphite", (0.12, 0.13, 0.14, 1.0))
    warm_gray = _material("warm_gray_plastic", (0.48, 0.49, 0.47, 1.0))
    metal = _material("brushed_metal", (0.56, 0.57, 0.55, 1.0))
    glass = _material("smoked_glass", (0.02, 0.03, 0.04, 0.72))
    lens = _material("blue_lens_glass", (0.04, 0.08, 0.14, 0.86))
    red = _material("red_nav_led", (1.0, 0.04, 0.02, 1.0))
    green = _material("green_nav_led", (0.02, 0.95, 0.15, 1.0))

    airframe = model.part("airframe")

    # Compact rounded central shell, with a raised battery/GNSS cover and real
    # side/front surface details to read as consumer/prosumer aircraft hardware.
    airframe.visual(
        _rounded_box_mesh((0.32, 0.215, 0.082), 0.032, "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=matte_black,
        name="body_shell",
    )
    airframe.visual(
        _rounded_box_mesh((0.205, 0.135, 0.026), 0.025, "top_battery_cover"),
        origin=Origin(xyz=(-0.015, 0.0, 0.223)),
        material=dark_gray,
        name="top_battery_cover",
    )
    airframe.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(-0.075, 0.0, 0.238)),
        material=warm_gray,
        name="gps_puck",
    )
    airframe.visual(
        Box((0.006, 0.082, 0.028)),
        origin=Origin(xyz=(0.163, 0.0, 0.173)),
        material=glass,
        name="front_sensor_window",
    )
    for slot_i, y in enumerate((-0.070, -0.047, 0.047, 0.070)):
        airframe.visual(
            Box((0.070, 0.009, 0.004)),
            origin=Origin(xyz=(-0.005, y, 0.238)),
            material=glass,
            name=f"cooling_slot_{slot_i}",
        )

    # Four broad, rigid carbon arms, each buried into the shell center and
    # motor pod so the airframe is one supported static assembly.
    for idx, (x, y) in enumerate(ROTOR_CENTERS):
        length = sqrt(x * x + y * y)
        yaw = atan2(y, x)
        airframe.visual(
            Box((length, 0.030, 0.020)),
            origin=Origin(xyz=(x * 0.50, y * 0.50, 0.181), rpy=(0.0, 0.0, yaw)),
            material=carbon,
            name=f"arm_{idx}",
        )
        airframe.visual(
            Cylinder(radius=0.052, length=0.047),
            origin=Origin(xyz=(x, y, 0.176)),
            material=dark_gray,
            name=f"motor_{idx}",
        )
        airframe.visual(
            Cylinder(radius=0.043, length=0.012),
            origin=Origin(xyz=(x, y, 0.205)),
            material=metal,
            name=f"motor_cap_{idx}",
        )
        airframe.visual(
            Cylinder(radius=0.058, length=0.006),
            origin=Origin(xyz=(x, y, 0.151)),
            material=matte_black,
            name=f"motor_foot_{idx}",
        )
        led_mat = red if y > 0.0 else green
        airframe.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x + (0.035 if x > 0.0 else -0.035), y, 0.194), rpy=(0.0, pi / 2.0, 0.0)),
            material=led_mat,
            name=f"nav_led_{idx}",
        )

    # Twin tubular landing skids with four struts and two cross-ties.  The rails
    # are slightly upturned at the tips, like a practical camera drone landing
    # gear rather than decorative floating rods.
    for side, y in enumerate((-0.145, 0.145)):
        rail = tube_from_spline_points(
            [
                (-0.225, y, 0.050),
                (-0.165, y, 0.037),
                (0.165, y, 0.037),
                (0.225, y, 0.050),
            ],
            radius=0.008,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
        airframe.visual(
            mesh_from_geometry(rail, f"skid_rail_{side}"),
            material=carbon,
            name=f"skid_rail_{side}",
        )
    for idx, (x, y) in enumerate(
        ((-0.125, -0.145), (0.125, -0.145), (-0.125, 0.145), (0.125, 0.145))
    ):
        airframe.visual(
            Cylinder(radius=0.0065, length=0.092),
            origin=Origin(xyz=(x, y, 0.085)),
            material=carbon,
            name=f"skid_strut_{idx}",
        )
    for idx, (x, y) in enumerate(
        ((-0.125, -0.126), (0.125, -0.126), (-0.125, 0.126), (0.125, 0.126))
    ):
        airframe.visual(
            Box((0.034, 0.058, 0.018)),
            origin=Origin(xyz=(x, y, 0.128)),
            material=dark_gray,
            name=f"skid_mount_{idx}",
        )
    for idx, x in enumerate((-0.125, 0.185)):
        airframe.visual(
            Box((0.026, 0.306, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.045)),
            material=carbon,
            name=f"skid_cross_tie_{idx}",
        )

    # Fixed lower gimbal mount integrated into the airframe.
    airframe.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(xyz=(0.112, 0.0, 0.118)),
        material=dark_gray,
        name="gimbal_mount",
    )

    # Exposed, independently spinning two-blade propeller assemblies.
    for idx, (x, y) in enumerate(ROTOR_CENTERS):
        propeller = model.part(f"propeller_{idx}")
        rotor = FanRotorGeometry(
            0.115,
            0.022,
            2,
            thickness=0.012,
            blade_pitch_deg=25.0,
            blade_sweep_deg=32.0,
            blade=FanRotorBlade(
                shape="scimitar",
                tip_pitch_deg=11.0,
                camber=0.14,
                tip_clearance=0.003,
            ),
            hub=FanRotorHub(
                style="spinner",
                rear_collar_height=0.006,
                rear_collar_radius=0.020,
                bore_diameter=0.006,
            ),
        )
        propeller.visual(
            mesh_from_geometry(rotor, f"propeller_{idx}_rotor"),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=warm_gray,
            name="rotor_disk",
        )
        propeller.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=metal,
            name="hub_nut",
        )
        propeller.visual(
            Cylinder(radius=0.007, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=metal,
            name="shaft",
        )
        model.articulation(
            f"motor_to_propeller_{idx}",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=propeller,
            origin=Origin(xyz=(x, y, 0.217)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=240.0),
        )

    # Two-axis underslung camera gimbal: yaw motor suspended from the body and a
    # pitch cradle with captured side pins on the camera block.
    gimbal_yaw = model.part("gimbal_yaw")
    gimbal_yaw.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_gray,
        name="yaw_motor",
    )
    gimbal_yaw.visual(
        Cylinder(radius=0.008, length=0.047),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=metal,
        name="drop_stem",
    )
    gimbal_yaw.visual(
        Box((0.020, 0.078, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=dark_gray,
        name="yoke_bridge",
    )
    for cheek_i, y in enumerate((0.034, -0.034)):
        gimbal_yaw.visual(
            Box((0.020, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, y, -0.092)),
            material=dark_gray,
            name=f"cheek_{cheek_i}",
        )

    model.articulation(
        "airframe_to_gimbal",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=gimbal_yaw,
        origin=Origin(xyz=(0.112, 0.0, 0.107)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-1.25, upper=1.25),
    )

    camera = model.part("camera")
    camera.visual(
        _rounded_box_mesh((0.052, 0.046, 0.037), 0.006, "camera_body"),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=matte_black,
        name="camera_body",
    )
    camera.visual(
        Cylinder(radius=0.017, length=0.017),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens,
        name="lens_glass",
    )
    for pin_i, y in enumerate((0.034, -0.034)):
        camera.visual(
            Cylinder(radius=0.006, length=0.022),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"pin_{pin_i}",
        )

    model.articulation(
        "gimbal_to_camera",
        ArticulationType.REVOLUTE,
        parent=gimbal_yaw,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-0.45, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    gimbal_yaw = object_model.get_part("gimbal_yaw")
    camera = object_model.get_part("camera")

    for idx in range(4):
        propeller = object_model.get_part(f"propeller_{idx}")
        ctx.allow_overlap(
            airframe,
            propeller,
            elem_a=f"motor_cap_{idx}",
            elem_b="shaft",
            reason="The exposed propeller shaft is intentionally captured in the motor cap bearing.",
        )
        ctx.expect_overlap(
            propeller,
            airframe,
            axes="xy",
            elem_a="shaft",
            elem_b=f"motor_cap_{idx}",
            min_overlap=0.006,
            name=f"propeller_{idx}_shaft_centered_in_motor",
        )
        ctx.expect_gap(
            propeller,
            airframe,
            axis="z",
            positive_elem="shaft",
            negative_elem=f"motor_cap_{idx}",
            max_penetration=0.016,
            name=f"propeller_{idx}_shaft_bearing_depth_limited",
        )

    for pin_i in range(2):
        ctx.allow_overlap(
            gimbal_yaw,
            camera,
            elem_a=f"cheek_{pin_i}",
            elem_b=f"pin_{pin_i}",
            reason="The camera trunnion pin is intentionally captured in the gimbal cheek bearing.",
        )
        ctx.expect_overlap(
            camera,
            gimbal_yaw,
            axes="xz",
            elem_a=f"pin_{pin_i}",
            elem_b=f"cheek_{pin_i}",
            min_overlap=0.006,
            name=f"camera_pin_{pin_i}_captured_by_yoke",
        )

    # Prove the visible gimbal mechanisms move in the expected directions.
    yaw = object_model.get_articulation("airframe_to_gimbal")
    pitch = object_model.get_articulation("gimbal_to_camera")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens_glass"))
    with ctx.pose({yaw: 0.70}):
        yawed_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens_glass"))
    ctx.check(
        "gimbal yaws the camera side to side",
        rest_lens is not None and yawed_lens is not None and yawed_lens[1] > rest_lens[1] + 0.015,
        details=f"rest_lens={rest_lens}, yawed_lens={yawed_lens}",
    )

    with ctx.pose({pitch: 0.75}):
        pitched_lens = _aabb_center(ctx.part_element_world_aabb(camera, elem="lens_glass"))
    ctx.check(
        "camera pitch joint tilts lens downward",
        rest_lens is not None and pitched_lens is not None and pitched_lens[2] < rest_lens[2] - 0.018,
        details=f"rest_lens={rest_lens}, pitched_lens={pitched_lens}",
    )

    ctx.check(
        "four independent propeller spin joints",
        all(object_model.get_articulation(f"motor_to_propeller_{idx}") is not None for idx in range(4)),
        details="Expected one continuous spin articulation at each motor pod.",
    )

    return ctx.report()


object_model = build_object_model()

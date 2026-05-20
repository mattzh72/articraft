from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_h_frame_quadrotor")

    carbon = Material("matte_carbon", rgba=(0.015, 0.017, 0.018, 1.0))
    carbon_edge = Material("carbon_cut_edge", rgba=(0.10, 0.11, 0.11, 1.0))
    dark_metal = Material("dark_anodized_motor", rgba=(0.045, 0.047, 0.050, 1.0))
    aluminum = Material("red_anodized_hardware", rgba=(0.75, 0.035, 0.025, 1.0))
    camera_black = Material("black_camera", rgba=(0.005, 0.005, 0.006, 1.0))
    glass = Material("camera_glass", rgba=(0.02, 0.06, 0.09, 1.0))
    prop_front = Material("front_orange_prop", rgba=(1.0, 0.32, 0.02, 0.88))
    prop_rear = Material("rear_smoke_prop", rgba=(0.07, 0.08, 0.085, 0.82))

    frame = model.part("frame")

    # A compact five-inch racing-quad layout: nose is +X, propeller axes are +Z.
    frame.visual(
        Box((0.18, 0.080, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=carbon,
        name="lower_plate",
    )
    for i, y in enumerate((-0.045, 0.045)):
        frame.visual(
            Box((0.225, 0.018, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.028)),
            material=carbon,
            name=f"top_rail_{i}",
        )
    for i, x in enumerate((-0.105, 0.0, 0.105)):
        frame.visual(
            Box((0.060, 0.108, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.028)),
            material=carbon,
            name=f"cross_plate_{i}",
        )

    # Red alloy standoffs visibly tie the upper H deck to the lower carbon plate.
    for ix, x in enumerate((-0.075, 0.075)):
        for iy, y in enumerate((-0.034, 0.034)):
            frame.visual(
                Cylinder(radius=0.004, length=0.020),
                origin=Origin(xyz=(x, y, 0.021)),
                material=aluminum,
                name=f"standoff_{ix}_{iy}",
            )

    # A small strapped flight stack/battery makes the central body read at quad scale.
    frame.visual(
        Box((0.105, 0.043, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, 0.041)),
        material=carbon_edge,
        name="battery",
    )
    frame.visual(
        Box((0.018, 0.061, 0.026)),
        origin=Origin(xyz=(-0.010, 0.0, 0.043)),
        material=aluminum,
        name="battery_strap",
    )

    # Four fixed diagonal carbon arms are part of the rigid H-frame assembly.
    motor_centers: list[tuple[float, float]] = [
        (0.170, 0.130),
        (0.170, -0.130),
        (-0.170, 0.130),
        (-0.170, -0.130),
    ]
    arm_roots: list[tuple[float, float]] = [
        (0.050, 0.034),
        (0.050, -0.034),
        (-0.050, 0.034),
        (-0.050, -0.034),
    ]
    for i, ((mx, my), (rx, ry)) in enumerate(zip(motor_centers, arm_roots)):
        dx = mx - rx
        dy = my - ry
        yaw = math.atan2(dy, dx)
        length = math.hypot(dx, dy) + 0.030
        frame.visual(
            Box((length, 0.018, 0.006)),
            origin=Origin(
                xyz=((mx + rx) * 0.5, (my + ry) * 0.5, 0.017),
                rpy=(0.0, 0.0, yaw),
            ),
            material=carbon,
            name=f"arm_{i}",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(mx, my, 0.021)),
            material=carbon,
            name=f"motor_mount_{i}",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(mx, my, 0.029)),
            material=dark_metal,
            name=f"motor_stator_{i}",
        )
        frame.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(mx, my, 0.037)),
            material=dark_metal,
            name=f"motor_bell_{i}",
        )

    # Nose yoke for the tilting camera plate.  The gap between cheeks captures
    # the camera hinge pin without hidden penetration at the neutral pose.
    frame.visual(
        Box((0.055, 0.076, 0.004)),
        origin=Origin(xyz=(0.122, 0.0, 0.028)),
        material=carbon,
        name="nose_plate",
    )
    frame.visual(
        Box((0.014, 0.006, 0.026)),
        origin=Origin(xyz=(0.128, -0.025, 0.033)),
        material=carbon,
        name="camera_yoke_0",
    )
    frame.visual(
        Box((0.014, 0.006, 0.026)),
        origin=Origin(xyz=(0.128, 0.025, 0.033)),
        material=carbon,
        name="camera_yoke_1",
    )

    prop_geom = FanRotorGeometry(
        outer_radius=0.063,
        hub_radius=0.012,
        blade_count=2,
        thickness=0.006,
        blade_pitch_deg=32.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.10),
        hub=FanRotorHub(style="spinner", bore_diameter=0.004),
    )
    for i, (mx, my) in enumerate(motor_centers):
        propeller = model.part(f"propeller_{i}")
        propeller.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark_metal,
            name="hub_collar",
        )
        propeller.visual(
            mesh_from_geometry(prop_geom, f"propeller_blades_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=prop_front if mx > 0.0 else prop_rear,
            name="blades",
        )
        model.articulation(
            f"propeller_axle_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=propeller,
            origin=Origin(xyz=(mx, my, 0.048)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=850.0),
        )

    camera_plate = model.part("camera_plate")
    camera_plate.visual(
        Cylinder(radius=0.005, length=0.044),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_pin",
    )
    camera_plate.visual(
        Box((0.010, 0.040, 0.032)),
        origin=Origin(xyz=(0.007, 0.0, -0.016)),
        material=carbon,
        name="plate",
    )
    camera_plate.visual(
        Box((0.008, 0.030, 0.020)),
        origin=Origin(xyz=(0.014, 0.0, -0.014)),
        material=camera_black,
        name="camera_body",
    )
    camera_plate.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.024, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens",
    )
    model.articulation(
        "camera_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=camera_plate,
        origin=Origin(xyz=(0.128, 0.0, 0.033)),
        # Positive command pitches the camera upward relative to the nose.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    camera = object_model.get_part("camera_plate")
    camera_hinge = object_model.get_articulation("camera_hinge")

    propeller_joints = [
        object_model.get_articulation(f"propeller_axle_{i}") for i in range(4)
    ]
    ctx.check(
        "four continuous propeller axles",
        len(propeller_joints) == 4
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in propeller_joints)
        and all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in propeller_joints),
        details=[(j.name, j.articulation_type, j.axis) for j in propeller_joints],
    )

    for i in range(4):
        propeller = object_model.get_part(f"propeller_{i}")
        ctx.expect_gap(
            propeller,
            frame,
            axis="z",
            positive_elem="hub_collar",
            negative_elem=f"motor_bell_{i}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"propeller_{i} hub sits on motor bell",
        )
        ctx.expect_overlap(
            propeller,
            frame,
            axes="xy",
            elem_a="hub_collar",
            elem_b=f"motor_bell_{i}",
            min_overlap=0.018,
            name=f"propeller_{i} axle is centered on motor",
        )

    ctx.expect_gap(
        frame,
        camera,
        axis="y",
        positive_elem="camera_yoke_1",
        negative_elem="hinge_pin",
        max_gap=0.001,
        max_penetration=0.0,
        name="positive yoke cheek touches hinge pin",
    )
    ctx.expect_gap(
        camera,
        frame,
        axis="y",
        positive_elem="hinge_pin",
        negative_elem="camera_yoke_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="negative yoke cheek touches hinge pin",
    )
    ctx.check(
        "camera hinge lies on nose centerline",
        abs(camera_hinge.origin.xyz[1]) < 1e-9
        and tuple(camera_hinge.axis) == (0.0, -1.0, 0.0)
        and camera_hinge.motion_limits is not None
        and camera_hinge.motion_limits.lower == -0.45
        and camera_hinge.motion_limits.upper == 0.45,
        details=f"origin={camera_hinge.origin}, axis={camera_hinge.axis}, limits={camera_hinge.motion_limits}",
    )

    def _coord(vec, index: int) -> float:
        if hasattr(vec, "__getitem__"):
            return float(vec[index])
        return float((vec.x, vec.y, vec.z)[index])

    def _center_z(aabb) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return (_coord(lo, 2) + _coord(hi, 2)) * 0.5

    rest_z = _center_z(ctx.part_element_world_aabb(camera, elem="lens"))
    with ctx.pose({camera_hinge: 0.45}):
        raised_z = _center_z(ctx.part_element_world_aabb(camera, elem="lens"))
    with ctx.pose({camera_hinge: -0.45}):
        lowered_z = _center_z(ctx.part_element_world_aabb(camera, elem="lens"))
    ctx.check(
        "camera plate tilts upward and downward",
        rest_z is not None
        and raised_z is not None
        and lowered_z is not None
        and raised_z > rest_z + 0.003
        and lowered_z < rest_z - 0.003,
        details=f"lowered={lowered_z}, rest={rest_z}, raised={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()

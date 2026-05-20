from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rehab_knee_scooter")

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.008, 0.008, 0.008, 1.0))
    cushion = model.material("soft_black_cushion", rgba=(0.01, 0.012, 0.014, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.50, 0.52, 0.54, 1.0))

    # Object frame: +X is the nose/front, +Y spans the pair of wheels, +Z is up.
    # The root part is deliberately one connected welded assembly: frame rail,
    # nose plate, handlebar mast, rear axle carrier, and fixed knee pad.
    frame = model.part("frame")
    frame.visual(
        Box((1.05, 0.075, 0.045)),
        origin=Origin(xyz=(-0.02, 0.0, 0.285)),
        material=graphite,
        name="main_rail",
    )
    frame.visual(
        Box((0.23, 0.43, 0.040)),
        origin=Origin(xyz=(0.515, 0.0, 0.245)),
        material=graphite,
        name="caster_nose_plate",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.760),
        origin=Origin(xyz=(0.445, 0.0, 0.645)),
        material=aluminum,
        name="handlebar_column",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.440),
        origin=Origin(xyz=(0.445, 0.0, 1.035), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="handlebar_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.120),
        origin=Origin(xyz=(0.445, 0.280, 1.035), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_0",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.120),
        origin=Origin(xyz=(0.445, -0.280, 1.035), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_1",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.540),
        origin=Origin(xyz=(-0.530, 0.0, 0.130), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="rear_axle_tube",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(-0.530, -0.264, 0.130), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="rear_washer_0",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(-0.530, 0.2735, 0.130), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="rear_washer_1",
    )
    frame.visual(
        Box((0.055, 0.060, 0.165)),
        origin=Origin(xyz=(-0.530, 0.0, 0.205)),
        material=graphite,
        name="rear_drop_block",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.105),
        origin=Origin(xyz=(-0.150, 0.0, 0.360)),
        material=aluminum,
        name="pad_post",
    )
    frame.visual(
        Box((0.390, 0.235, 0.055)),
        origin=Origin(xyz=(-0.150, 0.0, 0.430)),
        material=cushion,
        name="knee_pad",
    )
    for idx, y in enumerate((-0.135, 0.135)):
        frame.visual(
            Box((0.018, 0.058, 0.040)),
            origin=Origin(xyz=(0.506, y, 0.205)),
            material=aluminum,
            name=f"caster_socket_front_{idx}",
        )
        frame.visual(
            Box((0.018, 0.058, 0.040)),
            origin=Origin(xyz=(0.564, y, 0.205)),
            material=aluminum,
            name=f"caster_socket_rear_{idx}",
        )
        frame.visual(
            Box((0.058, 0.018, 0.040)),
            origin=Origin(xyz=(0.535, y - 0.029, 0.205)),
            material=aluminum,
            name=f"caster_socket_side_{idx}_0",
        )
        frame.visual(
            Box((0.058, 0.018, 0.040)),
            origin=Origin(xyz=(0.535, y + 0.029, 0.205)),
            material=aluminum,
            name=f"caster_socket_side_{idx}_1",
        )

    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.130,
            0.055,
            inner_radius=0.092,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.006, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        ),
        "rear_tire",
    )
    rear_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.096,
            0.047,
            rim=WheelRim(inner_radius=0.067, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.028,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.038, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "rear_wheel",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.045,
            inner_radius=0.050,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.0035, count=18, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.03),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )
    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.037,
            rim=WheelRim(inner_radius=0.036, flange_height=0.004, flange_thickness=0.0025),
            hub=WheelHub(radius=0.018, width=0.032, cap_style="flat"),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0028, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "caster_wheel",
    )

    for idx, y in enumerate((-0.300, 0.300)):
        wheel = model.part(f"rear_wheel_{idx}")
        wheel.visual(rear_tire_mesh, material=dark_rubber, name="tire")
        wheel.visual(rear_wheel_mesh, material=hub_gray, name="hub")
        model.articulation(
            f"rear_wheel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.530, y, 0.130), rpy=(0.0, 0.0, math.pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    # Two independent swiveling casters sit under the nose plate.  Each fork
    # turns on a vertical stem, and the small wheel spins on its own axle.
    for idx, y in enumerate((-0.135, 0.135)):
        caster = model.part(f"front_caster_{idx}")
        caster.visual(
            Cylinder(radius=0.014, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, -0.021)),
            material=aluminum,
            name="pivot_stem",
        )
        caster.visual(
            Box((0.074, 0.092, 0.024)),
            origin=Origin(xyz=(0.0, 0.0, -0.054)),
            material=aluminum,
            name="fork_crown",
        )
        caster.visual(
            Box((0.026, 0.010, 0.112)),
            origin=Origin(xyz=(-0.040, 0.039, -0.1215)),
            material=aluminum,
            name="fork_plate_0",
        )
        caster.visual(
            Box((0.026, 0.010, 0.112)),
            origin=Origin(xyz=(-0.040, -0.039, -0.1215)),
            material=aluminum,
            name="fork_plate_1",
        )
        caster.visual(
            Cylinder(radius=0.005, length=0.090),
            origin=Origin(xyz=(-0.040, 0.0, -0.150), rpy=(math.pi / 2, 0.0, 0.0)),
            material=aluminum,
            name="wheel_axle",
        )
        model.articulation(
            f"front_caster_{idx}_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(0.535, y, 0.225)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=8.0),
        )

        caster_wheel = model.part(f"caster_wheel_{idx}")
        caster_wheel.visual(caster_tire_mesh, material=dark_rubber, name="tire")
        caster_wheel.visual(caster_wheel_mesh, material=hub_gray, name="hub")
        model.articulation(
            f"caster_wheel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=caster_wheel,
            origin=Origin(xyz=(-0.040, 0.0, -0.150), rpy=(0.0, 0.0, math.pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    for idx in range(2):
        rear = object_model.get_part(f"rear_wheel_{idx}")
        caster = object_model.get_part(f"front_caster_{idx}")
        caster_wheel = object_model.get_part(f"caster_wheel_{idx}")
        rear_spin = object_model.get_articulation(f"rear_wheel_{idx}_spin")
        swivel = object_model.get_articulation(f"front_caster_{idx}_swivel")
        caster_spin = object_model.get_articulation(f"caster_wheel_{idx}_spin")

        ctx.allow_overlap(
            caster_wheel,
            caster,
            elem_a="hub",
            elem_b="wheel_axle",
            reason="The caster axle is intentionally captured through the simplified solid wheel hub/bearing proxy.",
        )

        ctx.check(
            f"rear wheel {idx} has spin joint",
            rear_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(rear_spin.axis) == (1.0, 0.0, 0.0),
            details=f"type={rear_spin.articulation_type}, axis={rear_spin.axis}",
        )
        ctx.check(
            f"front caster {idx} swivels vertically",
            swivel.articulation_type == ArticulationType.CONTINUOUS
            and tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"type={swivel.articulation_type}, axis={swivel.axis}",
        )
        ctx.check(
            f"caster wheel {idx} has spin joint",
            caster_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(caster_spin.axis) == (1.0, 0.0, 0.0),
            details=f"type={caster_spin.articulation_type}, axis={caster_spin.axis}",
        )

        ctx.expect_contact(
            frame,
            rear,
            elem_a=f"rear_washer_{idx}",
            elem_b="hub",
            contact_tol=0.003,
            name=f"rear wheel {idx} is carried on rear axle",
        )
        ctx.expect_contact(
            frame,
            caster,
            elem_a="caster_nose_plate",
            elem_b="pivot_stem",
            contact_tol=0.002,
            name=f"caster {idx} stem seats in bearing",
        )
        ctx.expect_gap(
            caster,
            caster_wheel,
            axis="y",
            positive_elem="fork_plate_0",
            negative_elem="tire",
            min_gap=0.004,
            max_gap=0.020,
            name=f"caster wheel {idx} clears outer fork plate",
        )
        ctx.expect_gap(
            caster_wheel,
            caster,
            axis="y",
            positive_elem="tire",
            negative_elem="fork_plate_1",
            min_gap=0.004,
            max_gap=0.020,
            name=f"caster wheel {idx} clears inner fork plate",
        )
        ctx.expect_contact(
            caster,
            caster_wheel,
            elem_a="wheel_axle",
            elem_b="hub",
            contact_tol=0.003,
            name=f"caster wheel {idx} rides on fork axle",
        )
        ctx.expect_within(
            caster,
            caster_wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="hub",
            margin=0.002,
            name=f"caster wheel {idx} axle is centered in hub",
        )
        ctx.expect_overlap(
            caster,
            caster_wheel,
            axes="y",
            elem_a="wheel_axle",
            elem_b="hub",
            min_overlap=0.030,
            name=f"caster wheel {idx} axle passes through hub",
        )

        rest_center = ctx.part_world_position(caster_wheel)
        with ctx.pose({swivel: math.pi / 2}):
            turned_center = ctx.part_world_position(caster_wheel)
        ctx.check(
            f"caster {idx} wheel follows swivel trail",
            rest_center is not None
            and turned_center is not None
            and abs(turned_center[1] - rest_center[1]) > 0.025,
            details=f"rest={rest_center}, turned={turned_center}",
        )

    return ctx.report()


object_model = build_object_model()

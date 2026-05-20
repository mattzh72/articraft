from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
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
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _z_cylinder() -> Origin:
    return Origin()


def _x_cylinder() -> Origin:
    return Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder() -> Origin:
    return Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _wheel_orientation() -> Origin:
    # Wheel helper meshes spin about local +X.  Rotate the mesh so that local
    # +X becomes the scooter's side-to-side +Y axle direction.
    return Origin(rpy=(0.0, 0.0, math.pi / 2.0))


def _pad_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.40, 0.22, 0.055, corner_segments=10),
            0.060,
            cap=True,
            center=True,
        ),
        "rounded_knee_pad",
    )


def _rear_wheel_meshes(index: int):
    rim = mesh_from_geometry(
        WheelGeometry(
            0.044,
            0.030,
            rim=WheelRim(
                inner_radius=0.030,
                flange_height=0.004,
                flange_thickness=0.003,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.017,
                width=0.026,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.025,
                    hole_diameter=0.003,
                ),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        f"rear_rim_{index}",
    )
    tire = mesh_from_geometry(
        TireGeometry(
            0.060,
            0.034,
            inner_radius=0.045,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.0025, count=18, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        f"rear_tire_{index}",
    )
    return rim, tire


def _front_wheel_meshes(index: int):
    rim = mesh_from_geometry(
        WheelGeometry(
            0.032,
            0.024,
            rim=WheelRim(
                inner_radius=0.022,
                flange_height=0.003,
                flange_thickness=0.002,
                bead_seat_depth=0.0015,
            ),
            hub=WheelHub(radius=0.014, width=0.021, cap_style="domed"),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=4, thickness=0.002, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        f"front_rim_{index}",
    )
    tire = mesh_from_geometry(
        TireGeometry(
            0.046,
            0.028,
            inner_radius=0.033,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.035),
            tread=TireTread(style="block", depth=0.002, count=16, land_ratio=0.65),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0012),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.003, radius=0.0015),
        ),
        f"front_tire_{index}",
    )
    return rim, tire


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_indoor_knee_scooter")

    satin_steel = model.material("satin_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    pad_blue = model.material("pad_blue", rgba=(0.02, 0.10, 0.22, 1.0))
    rim_gray = model.material("rim_gray", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")

    # Low welded tube frame: narrow rear, wider front, and a fixed handle post.
    frame.visual(
        Cylinder(radius=0.016, length=0.88),
        origin=Origin(xyz=(0.000, 0.000, 0.235), rpy=_x_cylinder().rpy),
        material=satin_steel,
        name="center_spine",
    )
    for i, y in enumerate((-0.135, 0.135)):
        frame.visual(
            Cylinder(radius=0.013, length=0.78),
            origin=Origin(xyz=(-0.045, y, 0.225), rpy=_x_cylinder().rpy),
            material=satin_steel,
            name=f"side_rail_{i}",
        )
    frame.visual(
        Cylinder(radius=0.017, length=0.39),
        origin=Origin(xyz=(0.375, 0.000, 0.238), rpy=_y_cylinder().rpy),
        material=satin_steel,
        name="front_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(-0.430, 0.000, 0.220), rpy=_y_cylinder().rpy),
        material=satin_steel,
        name="rear_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.50),
        origin=Origin(xyz=(-0.430, 0.000, 0.075), rpy=_y_cylinder().rpy),
        material=dark_steel,
        name="rear_axle",
    )
    for i, y in enumerate((-0.135, 0.135)):
        frame.visual(
            Cylinder(radius=0.010, length=0.150),
            origin=Origin(xyz=(-0.430, y, 0.148)),
            material=satin_steel,
            name=f"rear_dropout_{i}",
        )
        frame.visual(
            Box((0.070, 0.020, 0.020)),
            origin=Origin(xyz=(-0.430, y, 0.082)),
            material=satin_steel,
            name=f"axle_boss_{i}",
        )

    frame.visual(
        Box((0.34, 0.19, 0.016)),
        origin=Origin(xyz=(-0.090, 0.000, 0.285)),
        material=graphite,
        name="pad_plate",
    )
    for i, x in enumerate((-0.210, 0.030)):
        frame.visual(
            Cylinder(radius=0.014, length=0.060),
            origin=Origin(xyz=(x, 0.000, 0.260)),
            material=satin_steel,
            name=f"pad_standoff_{i}",
        )
    frame.visual(
        _pad_mesh(),
        origin=Origin(xyz=(-0.090, 0.000, 0.323)),
        material=pad_blue,
        name="knee_pad",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.670),
        origin=Origin(xyz=(0.375, 0.000, 0.570), rpy=_z_cylinder().rpy),
        material=satin_steel,
        name="handle_post",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.46),
        origin=Origin(xyz=(0.375, 0.000, 0.910), rpy=_y_cylinder().rpy),
        material=satin_steel,
        name="handlebar",
    )
    for i, y in enumerate((-0.255, 0.255)):
        frame.visual(
            Cylinder(radius=0.022, length=0.110),
            origin=Origin(xyz=(0.375, y, 0.910), rpy=_y_cylinder().rpy),
            material=black_rubber,
            name=f"grip_{i}",
        )
    frame.visual(
        Box((0.082, 0.050, 0.032)),
        origin=Origin(xyz=(0.375, 0.000, 0.265)),
        material=satin_steel,
        name="post_clamp",
    )

    # Each front caster has an open frame socket with a top and bottom retaining
    # collar on the swiveling stem.  The collars are larger than the socket hole
    # but clear the side tabs, so the caster remains clipped while free to turn.
    caster_x = 0.375
    caster_mount_z = 0.200
    for i, y in enumerate((-0.145, 0.145)):
        frame.visual(
            mesh_from_geometry(TorusGeometry(0.026, 0.006, radial_segments=32, tubular_segments=16), f"caster_socket_mesh_{i}"),
            origin=Origin(xyz=(caster_x, y, caster_mount_z)),
            material=satin_steel,
            name=f"caster_socket_{i}",
        )
        for j, dx in enumerate((-0.039, 0.039)):
            frame.visual(
                Box((0.014, 0.030, 0.046)),
                origin=Origin(xyz=(caster_x + (0.038 if dx > 0 else -0.038), y, caster_mount_z + 0.025)),
                material=satin_steel,
                name=f"socket_tab_{i}_{j}",
            )

    # Rear wheels: each wheel spins on the rigid rear axle support.
    for i, y in enumerate((-0.205, 0.205)):
        wheel = model.part(f"rear_wheel_{i}")
        rim_mesh, tire_mesh = _rear_wheel_meshes(i)
        wheel.visual(rim_mesh, origin=_wheel_orientation(), material=rim_gray, name="rim")
        wheel.visual(tire_mesh, origin=_wheel_orientation(), material=black_rubber, name="tire")
        model.articulation(
            f"frame_to_rear_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.430, y, 0.075)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=30.0),
        )

    # Front caster assemblies: vertical swivel at the frame socket, then an
    # independent wheel spin axis through each fork.
    for i, y in enumerate((-0.145, 0.145)):
        caster = model.part(f"front_caster_{i}")
        caster.visual(
            Cylinder(radius=0.007, length=0.106),
            origin=Origin(xyz=(0.000, 0.000, -0.036)),
            material=dark_steel,
            name="stem",
        )
        caster.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=Origin(xyz=(0.000, 0.000, 0.009)),
            material=dark_steel,
            name="upper_clip",
        )
        caster.visual(
            Cylinder(radius=0.025, length=0.006),
            origin=Origin(xyz=(0.000, 0.000, -0.009)),
            material=dark_steel,
            name="lower_clip",
        )
        caster.visual(
            Box((0.066, 0.066, 0.018)),
            origin=Origin(xyz=(-0.018, 0.000, -0.084)),
            material=satin_steel,
            name="fork_crown",
        )
        for j, side_y in enumerate((-0.026, 0.026)):
            caster.visual(
                Box((0.018, 0.006, 0.096)),
                origin=Origin(xyz=(-0.025, side_y, -0.138)),
                material=satin_steel,
                name=f"fork_cheek_{j}",
            )
        caster.visual(
            Cylinder(radius=0.007, length=0.076),
            origin=Origin(xyz=(-0.025, 0.000, -0.145), rpy=_y_cylinder().rpy),
            material=dark_steel,
            name="wheel_axle",
        )
        model.articulation(
            f"frame_to_front_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(caster_x, y, caster_mount_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=8.0),
        )

        front_wheel = model.part(f"front_wheel_{i}")
        rim_mesh, tire_mesh = _front_wheel_meshes(i)
        front_wheel.visual(rim_mesh, origin=_wheel_orientation(), material=rim_gray, name="rim")
        front_wheel.visual(tire_mesh, origin=_wheel_orientation(), material=black_rubber, name="tire")
        model.articulation(
            f"front_caster_{i}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=front_wheel,
            origin=Origin(xyz=(-0.025, 0.000, -0.145)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=35.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")

    for i in range(2):
        rear_joint = object_model.get_articulation(f"frame_to_rear_wheel_{i}")
        caster_joint = object_model.get_articulation(f"frame_to_front_caster_{i}")
        front_spin = object_model.get_articulation(f"front_caster_{i}_to_wheel")
        caster = object_model.get_part(f"front_caster_{i}")
        rear_wheel = object_model.get_part(f"rear_wheel_{i}")
        front_wheel = object_model.get_part(f"front_wheel_{i}")

        ctx.allow_overlap(
            frame,
            rear_wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The rear axle is intentionally modeled as the captured shaft passing through the wheel hub bearing.",
        )
        ctx.allow_overlap(
            caster,
            front_wheel,
            elem_a="wheel_axle",
            elem_b="rim",
            reason="The caster fork axle is intentionally modeled as the captured shaft through the small wheel hub bearing.",
        )

        ctx.check(
            f"rear wheel {i} spins on side axle",
            rear_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(rear_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={rear_joint.articulation_type}, axis={rear_joint.axis}",
        )
        ctx.check(
            f"front caster {i} swivels vertically",
            caster_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(caster_joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={caster_joint.articulation_type}, axis={caster_joint.axis}",
        )
        ctx.check(
            f"front caster wheel {i} spins on fork axle",
            front_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(front_spin.axis) == (0.0, 1.0, 0.0),
            details=f"type={front_spin.articulation_type}, axis={front_spin.axis}",
        )
        ctx.expect_within(
            frame,
            rear_wheel,
            axes="xz",
            inner_elem="rear_axle",
            outer_elem="rim",
            margin=0.0,
            name=f"rear axle {i} is centered in hub",
        )
        ctx.expect_overlap(
            frame,
            rear_wheel,
            axes="y",
            elem_a="rear_axle",
            elem_b="rim",
            min_overlap=0.024,
            name=f"rear axle {i} passes through wheel hub",
        )
        ctx.expect_within(
            caster,
            front_wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="rim",
            margin=0.0,
            name=f"caster axle {i} is centered in hub",
        )
        ctx.expect_overlap(
            caster,
            front_wheel,
            axes="y",
            elem_a="wheel_axle",
            elem_b="rim",
            min_overlap=0.018,
            name=f"caster axle {i} passes through wheel hub",
        )

        ctx.expect_overlap(
            caster,
            frame,
            axes="xy",
            elem_a="upper_clip",
            elem_b=f"caster_socket_{i}",
            min_overlap=0.010,
            name=f"caster {i} upper clip covers socket hole",
        )
        ctx.expect_gap(
            caster,
            frame,
            axis="z",
            positive_elem="upper_clip",
            negative_elem=f"caster_socket_{i}",
            max_gap=0.004,
            max_penetration=0.0005,
            name=f"caster {i} upper clip seats on socket",
        )
        ctx.expect_overlap(
            caster,
            frame,
            axes="xy",
            elem_a="lower_clip",
            elem_b=f"caster_socket_{i}",
            min_overlap=0.010,
            name=f"caster {i} lower clip covers socket hole",
        )
        ctx.expect_gap(
            frame,
            caster,
            axis="z",
            positive_elem=f"caster_socket_{i}",
            negative_elem="lower_clip",
            max_gap=0.004,
            max_penetration=0.0005,
            name=f"caster {i} lower clip retains socket",
        )

        with ctx.pose({caster_joint: 1.25}):
            ctx.expect_overlap(
                caster,
                frame,
                axes="xy",
                elem_a="upper_clip",
                elem_b=f"caster_socket_{i}",
                min_overlap=0.010,
                name=f"caster {i} remains clipped while swiveling",
            )

    return ctx.report()


object_model = build_object_model()

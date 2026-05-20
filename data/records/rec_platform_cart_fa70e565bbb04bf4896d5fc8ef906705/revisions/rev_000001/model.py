from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _x_cylinder(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin_x(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _origin_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_platform_cart")

    deck_blue = model.material("powder_coated_blue", rgba=(0.08, 0.22, 0.48, 1.0))
    tread_black = model.material("black_grip_mat", rgba=(0.015, 0.015, 0.014, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("dark_zinc_hardware", rgba=(0.12, 0.13, 0.13, 1.0))
    rubber = model.material("caster_rubber", rgba=(0.025, 0.024, 0.023, 1.0))
    rim_gray = model.material("pressed_steel_rim", rgba=(0.78, 0.78, 0.74, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((1.00, 0.60, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.92, 0.52, 0.010)),
        origin=Origin(xyz=(-0.02, 0.0, 0.274)),
        material=tread_black,
        name="grip_mat",
    )
    deck.visual(
        Box((1.04, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, 0.317, 0.287)),
        material=deck_blue,
        name="side_lip_0",
    )
    deck.visual(
        Box((1.04, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.317, 0.287)),
        material=deck_blue,
        name="side_lip_1",
    )
    deck.visual(
        Box((0.035, 0.60, 0.055)),
        origin=Origin(xyz=(0.515, 0.0, 0.282)),
        material=deck_blue,
        name="front_lip",
    )
    deck.visual(
        Box((0.035, 0.60, 0.040)),
        origin=Origin(xyz=(-0.515, 0.0, 0.280)),
        material=deck_blue,
        name="rear_lip",
    )

    caster_sites = (
        ("front_caster_0", "front_wheel_0", 0.36, 0.215, "front_mount_0"),
        ("front_caster_1", "front_wheel_1", 0.36, -0.215, "front_mount_1"),
        ("rear_caster_0", "rear_wheel_0", -0.36, 0.215, "rear_mount_0"),
        ("rear_caster_1", "rear_wheel_1", -0.36, -0.215, "rear_mount_1"),
    )

    for _, _, x, y, mount_name in caster_sites:
        deck.visual(
            Box((0.130, 0.105, 0.010)),
            origin=Origin(xyz=(x, y, 0.195)),
            material=steel,
            name=mount_name,
        )
        for bolt_i, (bx, by) in enumerate(
            ((-0.052, -0.039), (-0.052, 0.039), (0.052, -0.039), (0.052, 0.039))
        ):
            deck.visual(
                Cylinder(radius=0.008, length=0.005),
                origin=Origin(xyz=(x + bx, y + by, 0.1875)),
                material=dark_steel,
                name=f"{mount_name}_bolt_{bolt_i}",
            )

    deck.visual(
        Box((0.055, 0.030, 0.058)),
        origin=Origin(xyz=(0.555, 0.190, 0.330)),
        material=steel,
        name="handle_hinge_ear_0",
    )
    deck.visual(
        Box((0.055, 0.030, 0.058)),
        origin=Origin(xyz=(0.555, -0.190, 0.330)),
        material=steel,
        name="handle_hinge_ear_1",
    )

    handle = model.part("tow_loop")
    handle.visual(
        _x_cylinder(0.012, 0.330),
        origin=_origin_x(-0.165, 0.155, 0.0),
        material=steel,
        name="side_tube_0",
    )
    handle.visual(
        _x_cylinder(0.012, 0.330),
        origin=_origin_x(-0.165, -0.155, 0.0),
        material=steel,
        name="side_tube_1",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.350),
        origin=_origin_y(0.0, 0.0, 0.0),
        material=dark_steel,
        name="hinge_tube",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.310),
        origin=_origin_y(-0.330, 0.0, 0.0),
        material=steel,
        name="hand_grip",
    )
    handle.visual(
        Cylinder(radius=0.021, length=0.210),
        origin=_origin_y(-0.330, 0.0, 0.0),
        material=tread_black,
        name="rubber_grip",
    )

    model.articulation(
        "tow_loop_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(0.555, 0.0, 0.330)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=0.0, upper=1.70),
    )

    for caster_name, wheel_name, x, y, mount_name in caster_sites:
        caster = model.part(caster_name)
        caster.visual(
            Cylinder(radius=0.047, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=steel,
            name="swivel_plate",
        )
        caster.visual(
            Cylinder(radius=0.016, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.022)),
            material=dark_steel,
            name="kingpin",
        )
        caster.visual(
            Box((0.090, 0.052, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=steel,
            name="fork_crown",
        )
        caster.visual(
            Box((0.008, 0.052, 0.108)),
            origin=Origin(xyz=(0.033, 0.0, -0.101)),
            material=steel,
            name="fork_cheek_0",
        )
        caster.visual(
            Box((0.008, 0.052, 0.108)),
            origin=Origin(xyz=(-0.033, 0.0, -0.101)),
            material=steel,
            name="fork_cheek_1",
        )
        caster.visual(
            _x_cylinder(0.010, 0.080),
            origin=_origin_x(0.0, 0.0, -0.120),
            material=dark_steel,
            name="axle",
        )
        caster.visual(
            _x_cylinder(0.013, 0.012),
            origin=_origin_x(0.042, 0.0, -0.120),
            material=dark_steel,
            name="axle_cap_0",
        )
        caster.visual(
            _x_cylinder(0.013, 0.012),
            origin=_origin_x(-0.042, 0.0, -0.120),
            material=dark_steel,
            name="axle_cap_1",
        )

        model.articulation(
            f"{caster_name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x, y, 0.190)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=6.0),
        )

        wheel = model.part(wheel_name)
        tire = TireGeometry(
            0.070,
            0.045,
            inner_radius=0.046,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.0035, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.005, radius=0.002),
        )
        rim = WheelGeometry(
            0.048,
            0.038,
            rim=WheelRim(inner_radius=0.030, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.018,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.024, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.018),
        )
        wheel.visual(mesh_from_geometry(tire, f"{wheel_name}_tire"), material=rubber, name="tire")
        wheel.visual(mesh_from_geometry(rim, f"{wheel_name}_rim"), material=rim_gray, name="rim")

        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.120)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    handle = object_model.get_part("tow_loop")
    handle_joint = object_model.get_articulation("tow_loop_hinge")

    ctx.expect_contact(
        deck,
        handle,
        elem_a="handle_hinge_ear_0",
        elem_b="hinge_tube",
        contact_tol=0.002,
        name="tow loop hinge tube seats in one clevis ear",
    )
    ctx.expect_contact(
        deck,
        handle,
        elem_a="handle_hinge_ear_1",
        elem_b="hinge_tube",
        contact_tol=0.002,
        name="tow loop hinge tube seats in opposite clevis ear",
    )
    ctx.expect_gap(
        handle,
        deck,
        axis="z",
        positive_elem="side_tube_0",
        negative_elem="deck_shell",
        min_gap=0.035,
        name="folded tow loop clears the deck surface",
    )

    if handle_joint.motion_limits is not None and handle_joint.motion_limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(handle)
        with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
            raised_aabb = ctx.part_world_aabb(handle)

        def _zmax(aabb):
            if aabb is None:
                return None
            high = aabb[1]
            return high[2] if hasattr(high, "__getitem__") else high.z

        rest_z = _zmax(rest_aabb)
        raised_z = _zmax(raised_aabb)
        ctx.check(
            "tow loop folds upward from the front hinge",
            rest_z is not None and raised_z is not None and raised_z > rest_z + 0.20,
            details=f"rest_z={rest_z}, raised_z={raised_z}",
        )
    else:
        ctx.fail("tow loop has an upper folding stop", "missing revolute motion limits")

    caster_sites = (
        ("front_caster_0", "front_wheel_0", "front_mount_0"),
        ("front_caster_1", "front_wheel_1", "front_mount_1"),
        ("rear_caster_0", "rear_wheel_0", "rear_mount_0"),
        ("rear_caster_1", "rear_wheel_1", "rear_mount_1"),
    )

    for caster_name, wheel_name, mount_name in caster_sites:
        caster = object_model.get_part(caster_name)
        wheel = object_model.get_part(wheel_name)
        swivel = object_model.get_articulation(f"{caster_name}_swivel")
        spin = object_model.get_articulation(f"{wheel_name}_spin")

        ctx.check(
            f"{caster_name} swivels about a vertical kingpin",
            swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"type={swivel.articulation_type}, axis={swivel.axis}",
        )
        ctx.check(
            f"{wheel_name} spins on a horizontal axle",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )
        ctx.expect_contact(
            deck,
            caster,
            elem_a=mount_name,
            elem_b="swivel_plate",
            contact_tol=0.001,
            name=f"{caster_name} swivel plate is mounted to the deck",
        )
        ctx.expect_within(
            wheel,
            caster,
            axes="x",
            inner_elem="tire",
            outer_elem="fork_crown",
            margin=0.0,
            name=f"{wheel_name} sits between the caster fork cheeks",
        )

        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The caster axle is intentionally modeled as a captured shaft passing through the wheel hub.",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="x",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.030,
            name=f"{wheel_name} rim remains captured on its axle",
        )
        ctx.expect_within(
            caster,
            wheel,
            axes="yz",
            inner_elem="axle",
            outer_elem="rim",
            margin=0.002,
            name=f"{wheel_name} axle is centered inside the rim bore",
        )

    return ctx.report()


object_model = build_object_model()

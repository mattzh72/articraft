from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
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
    model = ArticulatedObject(name="fold_handle_platform_trolley")

    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = Material("dark_galvanized_steel", rgba=(0.22, 0.24, 0.25, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    blue_grey = Material("blue_grey_powdercoat", rgba=(0.18, 0.28, 0.38, 1.0))
    bolt_metal = Material("bolt_heads", rgba=(0.48, 0.49, 0.47, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.80, 0.50, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=steel,
        name="steel_deck",
    )
    deck.visual(
        Box((0.70, 0.40, 0.006)),
        origin=Origin(xyz=(0.030, 0.0, 0.1965)),
        material=black_rubber,
        name="rubber_tread_mat",
    )
    # Folded flanges and underside ribs make the thin platform read as a pressed steel deck.
    deck.visual(
        Box((0.82, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, 0.263, 0.158)),
        material=steel,
        name="side_lip_0",
    )
    deck.visual(
        Box((0.82, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -0.263, 0.158)),
        material=steel,
        name="side_lip_1",
    )
    deck.visual(
        Box((0.030, 0.50, 0.040)),
        origin=Origin(xyz=(0.415, 0.0, 0.158)),
        material=steel,
        name="front_lip",
    )
    deck.visual(
        Box((0.030, 0.50, 0.040)),
        origin=Origin(xyz=(-0.415, 0.0, 0.158)),
        material=steel,
        name="rear_lip",
    )
    for y, name in ((0.145, "rib_0"), (-0.145, "rib_1")):
        deck.visual(
            Box((0.66, 0.035, 0.026)),
            origin=Origin(xyz=(0.02, y, 0.134)),
            material=dark_steel,
            name=name,
        )

    caster_positions = (
        (0.305, 0.185),
        (0.305, -0.185),
        (-0.305, 0.185),
        (-0.305, -0.185),
    )
    for idx, (x, y) in enumerate(caster_positions):
        deck.visual(
            Box((0.145, 0.120, 0.008)),
            origin=Origin(xyz=(x, y, 0.1438)),
            material=dark_steel,
            name=f"caster_mount_{idx}",
        )
        for bx in (-0.045, 0.045):
            for by in (-0.035, 0.035):
                deck.visual(
                    Cylinder(0.006, 0.004),
                    origin=Origin(xyz=(x + bx, y + by, 0.1495)),
                    material=bolt_metal,
                    name=f"caster_bolt_{idx}_{bx}_{by}",
                )

    # Fixed clevis leaves for the two rear side hinges.  The moving handle lugs sit between
    # these ears, all on one common horizontal hinge axis.
    hinge_x = -0.392
    hinge_z = 0.221
    hinge_side_y = 0.180
    for side_idx, side in enumerate((hinge_side_y, -hinge_side_y)):
        deck.visual(
            Box((0.095, 0.080, 0.007)),
            origin=Origin(xyz=(hinge_x + 0.025, side, 0.192)),
            material=dark_steel,
            name=f"hinge_leaf_{side_idx}",
        )
        sign = 1.0 if side > 0.0 else -1.0
        for ear_idx, offset in enumerate((0.0275, -0.0275)):
            deck.visual(
                Box((0.055, 0.005, 0.070)),
                origin=Origin(xyz=(hinge_x, side + sign * offset, hinge_z)),
                material=dark_steel,
                name=f"hinge_ear_{side_idx}_{ear_idx}",
            )
        deck.visual(
            Cylinder(0.010, 0.062),
            origin=Origin(
                xyz=(hinge_x, side, hinge_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=bolt_metal,
            name=f"hinge_pin_cap_{side_idx}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(0.018, 0.640),
        origin=Origin(xyz=(0.0, hinge_side_y, 0.360)),
        material=blue_grey,
        name="side_tube_0",
    )
    handle.visual(
        Cylinder(0.018, 0.640),
        origin=Origin(xyz=(0.0, -hinge_side_y, 0.360)),
        material=blue_grey,
        name="side_tube_1",
    )
    handle.visual(
        Cylinder(0.018, 2.0 * hinge_side_y),
        origin=Origin(
            xyz=(0.0, 0.0, 0.680),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=blue_grey,
        name="top_grip",
    )
    for side_idx, side in enumerate((hinge_side_y, -hinge_side_y)):
        handle.visual(
            Sphere(0.019),
            origin=Origin(xyz=(0.0, side, 0.680)),
            material=blue_grey,
            name=f"grip_bend_{side_idx}",
        )
        handle.visual(
            Cylinder(0.023, 0.050),
            origin=Origin(
                xyz=(0.0, side, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=blue_grey,
            name=f"hinge_barrel_{side_idx}",
        )
        handle.visual(
            Sphere(0.020),
            origin=Origin(xyz=(0.0, side, 0.035)),
            material=blue_grey,
            name=f"lower_tube_bend_{side_idx}",
        )
    handle.visual(
        Cylinder(0.010, 0.340),
        origin=Origin(
            xyz=(0.0, 0.0, 0.170),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=blue_grey,
        name="lower_cross_tie",
    )

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=math.pi / 2.0),
        motion_properties=MotionProperties(damping=0.25, friction=0.06),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.036,
            0.034,
            rim=WheelRim(inner_radius=0.024, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.015,
                width=0.024,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.020, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "small_caster_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.050,
            0.036,
            inner_radius=0.035,
            tread=TireTread(style="ribbed", depth=0.003, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "small_caster_tire",
    )

    swivel_z = 0.1398
    wheel_center_z = -0.0878
    for idx, (x, y) in enumerate(caster_positions):
        fork = model.part(f"caster_fork_{idx}")
        fork.visual(
            Cylinder(0.016, 0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=bolt_metal,
            name="swivel_stem",
        )
        fork.visual(
            Cylinder(0.032, 0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.033)),
            material=dark_steel,
            name="swivel_bearing",
        )
        fork.visual(
            Box((0.026, 0.024, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=dark_steel,
            name="fork_neck",
        )
        fork.visual(
            Box((0.095, 0.034, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=dark_steel,
            name="fork_crown",
        )
        for side, side_name in ((0.031, "fork_arm_0"), (-0.031, "fork_arm_1")):
            fork.visual(
                Box((0.010, 0.032, 0.108)),
                origin=Origin(xyz=(side, 0.0, -0.080)),
                material=dark_steel,
                name=side_name,
            )
        fork.visual(
            Cylinder(0.007, 0.086),
            origin=Origin(
                xyz=(0.0, 0.0, wheel_center_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_metal,
            name="axle_pin",
        )

        wheel = model.part(f"wheel_{idx}")
        wheel.visual(wheel_mesh, material=steel, name="rim")
        wheel.visual(tire_mesh, material=black_rubber, name="tire")
        wheel.visual(
            Cylinder(0.009, 0.050),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_metal,
            name="hub_sleeve",
        )

        model.articulation(
            f"caster_swivel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, swivel_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
            motion_properties=MotionProperties(damping=0.04, friction=0.02),
        )
        model.articulation(
            f"wheel_axle_{idx}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, wheel_center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=30.0),
            motion_properties=MotionProperties(damping=0.01, friction=0.005),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    handle_hinge = object_model.get_articulation("handle_hinge")

    for side_idx in range(2):
        ctx.allow_overlap(
            deck,
            handle,
            elem_a=f"hinge_pin_cap_{side_idx}",
            elem_b=f"hinge_barrel_{side_idx}",
            reason="The visible hinge pin is intentionally captured through the folding handle barrel.",
        )
        ctx.expect_overlap(
            deck,
            handle,
            axes="y",
            elem_a=f"hinge_pin_cap_{side_idx}",
            elem_b=f"hinge_barrel_{side_idx}",
            min_overlap=0.02,
            name=f"hinge pin {side_idx} passes through handle barrel",
        )

    ctx.expect_overlap(
        handle,
        deck,
        axes="y",
        elem_a="hinge_barrel_0",
        elem_b="hinge_ear_0_0",
        min_overlap=0.0,
        name="upper side hinge is aligned with deck clevis",
    )
    ctx.expect_overlap(
        handle,
        deck,
        axes="y",
        elem_a="hinge_barrel_1",
        elem_b="hinge_ear_1_0",
        min_overlap=0.0,
        name="lower side hinge is aligned with deck clevis",
    )

    upright_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_hinge: math.pi / 2.0}):
        ctx.expect_gap(
            handle,
            deck,
            axis="z",
            positive_elem="side_tube_0",
            negative_elem="rubber_tread_mat",
            min_gap=0.001,
            name="folded handle tube lies just above the deck mat",
        )
        ctx.expect_overlap(
            handle,
            deck,
            axes="xy",
            min_overlap=0.25,
            name="folded handle rests over the platform footprint",
        )
        folded_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "handle folds from upright to flat",
        upright_aabb is not None
        and folded_aabb is not None
        and upright_aabb[1][2] > 0.85
        and (folded_aabb[1][2] - folded_aabb[0][2]) < 0.09,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    for idx in range(4):
        fork = object_model.get_part(f"caster_fork_{idx}")
        wheel = object_model.get_part(f"wheel_{idx}")
        swivel = object_model.get_articulation(f"caster_swivel_{idx}")
        axle = object_model.get_articulation(f"wheel_axle_{idx}")

        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="axle_pin",
            elem_b="hub_sleeve",
            reason="The caster axle pin intentionally passes through the rotating wheel hub sleeve.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="x",
            elem_a="axle_pin",
            elem_b="hub_sleeve",
            min_overlap=0.04,
            name=f"wheel {idx} hub is captured by its axle",
        )
        ctx.check(
            f"caster {idx} has vertical swivel",
            tuple(round(v, 6) for v in swivel.axis) == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"wheel {idx} rotates on horizontal axle",
            tuple(round(v, 6) for v in axle.axis) == (1.0, 0.0, 0.0),
            details=f"axis={axle.axis}",
        )
        ctx.expect_contact(
            fork,
            deck,
            elem_a="swivel_stem",
            elem_b=f"caster_mount_{idx}",
            contact_tol=0.001,
            name=f"caster {idx} stem seats under mounting plate",
        )
        ctx.expect_within(
            wheel,
            fork,
            axes="x",
            inner_elem="tire",
            outer_elem="fork_crown",
            margin=0.0,
            name=f"wheel {idx} is centered between fork sides",
        )
        ctx.expect_gap(
            fork,
            wheel,
            axis="x",
            positive_elem="fork_arm_0",
            negative_elem="tire",
            min_gap=0.005,
            name=f"wheel {idx} clears one fork arm",
        )

    return ctx.report()


object_model = build_object_model()

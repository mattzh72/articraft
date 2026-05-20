from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _cyl_y(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _add_caster_fork(part, *, steel: Material, dark: Material, name_prefix: str) -> None:
    """Swivel caster yoke, authored with the pivot at the top mounting plane."""
    # Vertical kingpin and bearing stack, all one touching metal assembly.
    part.visual(
        Cylinder(radius=0.058, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=dark,
        name=f"{name_prefix}_swivel_bearing",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=steel,
        name=f"{name_prefix}_kingpin",
    )
    part.visual(
        Box((0.125, 0.150, 0.038)),
        origin=Origin(xyz=(-0.070, 0.0, -0.075)),
        material=steel,
        name=f"{name_prefix}_fork_bridge",
    )
    for side_index, y in enumerate((-0.058, 0.058)):
        part.visual(
            Box((0.040, 0.018, 0.172)),
            origin=Origin(xyz=(-0.075, y, -0.180)),
            material=steel,
            name=f"{name_prefix}_fork_arm_{side_index}",
        )
    part.visual(
        _cyl_y(0.013, 0.150),
        origin=Origin(xyz=(-0.075, 0.0, -0.190), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"{name_prefix}_axle_pin",
    )


def _add_caster_wheel(part, *, rubber: Material, rim_mat: Material, dark: Material, name_prefix: str) -> None:
    """Small solid polyurethane caster wheel with steel side caps."""
    # Cylinders are aligned along local +X so the child joint can spin about X.
    part.visual(
        Cylinder(radius=0.088, length=0.070),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.084),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(-0.039, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_mat,
        name="side_cap_0",
    )
    part.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_mat,
        name="side_cap_1",
    )


def _add_push_frame(part, *, steel: Material, grip: Material) -> None:
    """One welded tubular rear push frame, child frame at the hinge line."""
    main_loop = wire_from_points(
        [
            (0.0, -0.335, 0.035),
            (0.0, -0.335, 1.20),
            (0.0, -0.285, 1.34),
            (0.0, 0.285, 1.34),
            (0.0, 0.335, 1.20),
            (0.0, 0.335, 0.035),
        ],
        radius=0.028,
        radial_segments=20,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.060,
        corner_segments=8,
    )
    part.visual(mesh_from_geometry(main_loop, "push_frame_tube"), material=steel, name="tube_loop")

    lower_cross = tube_from_spline_points(
        [(0.0, -0.335, 0.28), (0.0, 0.335, 0.28)],
        radius=0.021,
        samples_per_segment=4,
        radial_segments=18,
    )
    part.visual(mesh_from_geometry(lower_cross, "push_frame_lower_crossbar"), material=steel, name="lower_crossbar")

    diagonal_0 = tube_from_spline_points(
        [(0.0, -0.335, 0.14), (0.0, 0.0, 0.68), (0.0, 0.335, 1.08)],
        radius=0.017,
        samples_per_segment=10,
        radial_segments=16,
    )
    diagonal_1 = tube_from_spline_points(
        [(0.0, 0.335, 0.14), (0.0, 0.0, 0.68), (0.0, -0.335, 1.08)],
        radius=0.017,
        samples_per_segment=10,
        radial_segments=16,
    )
    part.visual(mesh_from_geometry(diagonal_0, "push_frame_diagonal_0"), material=steel, name="diagonal_0")
    part.visual(mesh_from_geometry(diagonal_1, "push_frame_diagonal_1"), material=steel, name="diagonal_1")

    part.visual(
        Cylinder(radius=0.038, length=0.128),
        origin=Origin(xyz=(0.0, -0.335, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve_0",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.128),
        origin=Origin(xyz=(0.0, 0.335, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve_1",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 1.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip,
        name="hand_grip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_lumber_cart")

    steel = model.material("powder_coated_steel", rgba=(0.10, 0.14, 0.16, 1.0))
    dark_steel = model.material("dark_hardware", rgba=(0.025, 0.028, 0.030, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    orange_poly = model.material("orange_polyurethane", rgba=(0.95, 0.38, 0.08, 1.0))
    lumber_deck = model.material("sealed_wood_planks", rgba=(0.62, 0.42, 0.23, 1.0))
    grip_mat = model.material("matte_grip", rgba=(0.06, 0.065, 0.060, 1.0))

    deck = model.part("deck")
    # Low, long steel tray with wood boards inset into a welded perimeter.
    deck.visual(Box((2.42, 0.76, 0.070)), origin=Origin(xyz=(0.0, 0.0, 0.310)), material=steel, name="steel_pan")
    for i, y in enumerate((-0.285, -0.095, 0.095, 0.285)):
        deck.visual(
            Box((2.26, 0.168, 0.028)),
            origin=Origin(xyz=(0.015, y, 0.358)),
            material=lumber_deck,
            name=f"deck_plank_{i}",
        )
    for y, name in ((-0.405, "side_rail_0"), (0.405, "side_rail_1")):
        deck.visual(Box((2.48, 0.045, 0.108)), origin=Origin(xyz=(0.0, y, 0.326)), material=steel, name=name)
    for x, name in ((1.230, "front_bumper"), (-1.230, "rear_bumper")):
        deck.visual(Box((0.050, 0.850, 0.108)), origin=Origin(xyz=(x, 0.0, 0.326)), material=steel, name=name)
    deck.visual(Box((0.050, 0.780, 0.105)), origin=Origin(xyz=(1.165, 0.0, 0.424)), material=steel, name="front_stop_lip")

    # Swivel caster mounting plates touch the deck pan underside.
    caster_mounts = [
        ("front_0", 0.990, -0.285),
        ("front_1", 0.990, 0.285),
        ("rear_0", -0.990, -0.285),
        ("rear_1", -0.990, 0.285),
    ]
    for mount_name, x, y in caster_mounts:
        deck.visual(
            Box((0.205, 0.165, 0.030)),
            origin=Origin(xyz=(x, y, 0.260)),
            material=galvanized,
            name=f"{mount_name}_caster_plate",
        )
        deck.visual(
            Cylinder(radius=0.050, length=0.018),
            origin=Origin(xyz=(x, y, 0.247)),
            material=dark_steel,
            name=f"{mount_name}_swivel_race",
        )

    # Exposed rear hinge pin and leaf fixed to the deck rear edge.
    deck.visual(
        Cylinder(radius=0.023, length=0.830),
        origin=Origin(xyz=(-1.300, 0.0, 0.418), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="rear_hinge_pin",
    )
    deck.visual(
        Box((0.060, 0.740, 0.016)),
        origin=Origin(xyz=(-1.218, 0.0, 0.370)),
        material=galvanized,
        name="rear_hinge_leaf",
    )
    deck.visual(
        Box((0.064, 0.110, 0.060)),
        origin=Origin(xyz=(-1.278, 0.0, 0.400)),
        material=galvanized,
        name="rear_hinge_center_tab",
    )

    push_frame = model.part("push_frame")
    _add_push_frame(push_frame, steel=galvanized, grip=grip_mat)

    model.articulation(
        "deck_to_push_frame",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=push_frame,
        origin=Origin(xyz=(-1.300, 0.0, 0.418)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.1, lower=0.0, upper=1.48),
    )

    for caster_name, x, y in caster_mounts:
        fork = model.part(f"{caster_name}_fork")
        _add_caster_fork(fork, steel=galvanized, dark=dark_steel, name_prefix=caster_name)
        wheel = model.part(f"{caster_name}_wheel")
        _add_caster_wheel(wheel, rubber=orange_poly, rim_mat=galvanized, dark=dark_steel, name_prefix=caster_name)

        model.articulation(
            f"deck_to_{caster_name}_fork",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=fork,
            origin=Origin(xyz=(x, y, 0.238)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        )
        model.articulation(
            f"{caster_name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-0.075, 0.0, -0.190), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=35.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    push_frame = object_model.get_part("push_frame")
    frame_hinge = object_model.get_articulation("deck_to_push_frame")

    # The rear frame sleeves are deliberately modeled as captured around the
    # fixed hinge pin; the overlap is local and hidden inside the hinge barrel.
    for sleeve in ("hinge_sleeve_0", "hinge_sleeve_1"):
        ctx.allow_overlap(
            deck,
            push_frame,
            elem_a="rear_hinge_pin",
            elem_b=sleeve,
            reason="The push-frame hinge sleeve is intentionally captured around the rear hinge pin.",
        )
        ctx.expect_overlap(
            deck,
            push_frame,
            axes="y",
            elem_a="rear_hinge_pin",
            elem_b=sleeve,
            min_overlap=0.10,
            name=f"{sleeve} retained on hinge pin",
        )
        ctx.expect_gap(
            push_frame,
            deck,
            axis="z",
            positive_elem=sleeve,
            negative_elem="rear_hinge_pin",
            max_penetration=0.065,
            name=f"{sleeve} concentric with hinge pin",
        )

    grip_rest_aabb = ctx.part_element_world_aabb(push_frame, elem="hand_grip")
    rest_grip_x = None if grip_rest_aabb is None else (grip_rest_aabb[0][0] + grip_rest_aabb[1][0]) * 0.5
    with ctx.pose({frame_hinge: 1.45}):
        grip_folded_aabb = ctx.part_element_world_aabb(push_frame, elem="hand_grip")
        folded_grip_x = None if grip_folded_aabb is None else (grip_folded_aabb[0][0] + grip_folded_aabb[1][0]) * 0.5
        ctx.expect_overlap(
            push_frame,
            deck,
            axes="x",
            elem_a="hand_grip",
            elem_b="steel_pan",
            min_overlap=0.04,
            name="folded push frame reaches over deck",
        )
    ctx.check(
        "push frame folds forward from rear hinge",
        rest_grip_x is not None and folded_grip_x is not None and folded_grip_x > rest_grip_x + 0.80,
        details=f"rest_grip_x={rest_grip_x}, folded_grip_x={folded_grip_x}",
    )

    for caster_name in ("front_0", "front_1", "rear_0", "rear_1"):
        fork = object_model.get_part(f"{caster_name}_fork")
        wheel = object_model.get_part(f"{caster_name}_wheel")
        swivel = object_model.get_articulation(f"deck_to_{caster_name}_fork")
        spin = object_model.get_articulation(f"{caster_name}_wheel_spin")
        ctx.expect_contact(
            deck,
            fork,
            elem_a=f"{caster_name}_swivel_race",
            elem_b=f"{caster_name}_swivel_bearing",
            contact_tol=0.004,
            name=f"{caster_name} fork is mounted under deck",
        )
        ctx.expect_within(
            wheel,
            fork,
            axes="y",
            inner_elem="tire",
            outer_elem=f"{caster_name}_fork_bridge",
            margin=0.020,
            name=f"{caster_name} wheel sits between fork arms",
        )
        ctx.check(
            f"{caster_name} caster has swivel and spin",
            swivel.articulation_type == ArticulationType.CONTINUOUS
            and spin.articulation_type == ArticulationType.CONTINUOUS,
            details=f"swivel={swivel.articulation_type}, spin={spin.articulation_type}",
        )
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a=f"{caster_name}_axle_pin",
            elem_b="hub_core",
            reason="The solid hub proxy is intentionally captured around the fixed caster axle pin.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="y",
            elem_a=f"{caster_name}_axle_pin",
            elem_b="hub_core",
            min_overlap=0.070,
            name=f"{caster_name} axle passes through wheel hub",
        )
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a=f"{caster_name}_axle_pin",
            elem_b="tire",
            reason="The caster axle also passes through the simplified solid polyurethane wheel body.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="y",
            elem_a=f"{caster_name}_axle_pin",
            elem_b="tire",
            min_overlap=0.060,
            name=f"{caster_name} axle retained through tire body",
        )
        for cap in ("side_cap_0", "side_cap_1"):
            ctx.allow_overlap(
                fork,
                wheel,
                elem_a=f"{caster_name}_axle_pin",
                elem_b=cap,
                reason="The fixed caster axle is intentionally captured through the steel side cap.",
            )
            ctx.expect_overlap(
                fork,
                wheel,
                axes="y",
                elem_a=f"{caster_name}_axle_pin",
                elem_b=cap,
                min_overlap=0.006,
                name=f"{caster_name} axle retained through {cap}",
            )

    return ctx.report()


object_model = build_object_model()

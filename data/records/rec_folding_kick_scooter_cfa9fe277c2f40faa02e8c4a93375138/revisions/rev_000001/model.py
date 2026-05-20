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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGroove,
    TireGeometry,
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
    rounded_rect_profile,
)


def _tube_geometry(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    z0: float = 0.0,
    segments: int = 40,
) -> MeshGeometry:
    """Thin-walled circular tube aligned with local +Z."""

    vertices: list[tuple[float, float, float]] = []

    for z in (z0, z0 + length):
        for radius in (outer_radius, inner_radius):
            for i in range(segments):
                a = 2.0 * math.pi * i / segments
                vertices.append((radius * math.cos(a), radius * math.sin(a), z))

    def idx(layer: int, ring: int, i: int) -> int:
        # layer: 0 bottom, 1 top; ring: 0 outer, 1 inner
        return layer * (2 * segments) + ring * segments + (i % segments)

    faces: list[tuple[int, int, int]] = []
    for i in range(segments):
        j = (i + 1) % segments

        # Outer wall.
        faces.append((idx(0, 0, i), idx(0, 0, j), idx(1, 0, j)))
        faces.append((idx(0, 0, i), idx(1, 0, j), idx(1, 0, i)))

        # Inner wall, winding reversed.
        faces.append((idx(0, 1, j), idx(0, 1, i), idx(1, 1, i)))
        faces.append((idx(0, 1, j), idx(1, 1, i), idx(1, 1, j)))

        # Bottom annulus.
        faces.append((idx(0, 1, i), idx(0, 1, j), idx(0, 0, j)))
        faces.append((idx(0, 1, i), idx(0, 0, j), idx(0, 0, i)))

        # Top annulus.
        faces.append((idx(1, 0, i), idx(1, 0, j), idx(1, 1, j)))
        faces.append((idx(1, 0, i), idx(1, 1, j), idx(1, 1, i)))

    return MeshGeometry(vertices=vertices, faces=faces)


def _arched_fender_geometry(
    *,
    center_x: float,
    center_z: float,
    inner_radius: float,
    thickness: float,
    width: float,
    start_deg: float,
    end_deg: float,
    segments: int = 28,
) -> MeshGeometry:
    """A curved rear-fender shell: arc in XZ, width along Y."""

    vertices: list[tuple[float, float, float]] = []
    radii = (inner_radius, inner_radius + thickness)
    ys = (-width / 2.0, width / 2.0)
    for radius in radii:
        for y in ys:
            for i in range(segments + 1):
                t = math.radians(start_deg + (end_deg - start_deg) * i / segments)
                vertices.append((center_x + radius * math.cos(t), y, center_z + radius * math.sin(t)))

    def idx(ring: int, side: int, i: int) -> int:
        return ring * (2 * (segments + 1)) + side * (segments + 1) + i

    faces: list[tuple[int, int, int]] = []
    for i in range(segments):
        j = i + 1
        # Inner and outer curved faces.
        faces.append((idx(0, 0, i), idx(0, 0, j), idx(0, 1, j)))
        faces.append((idx(0, 0, i), idx(0, 1, j), idx(0, 1, i)))
        faces.append((idx(1, 0, i), idx(1, 1, i), idx(1, 1, j)))
        faces.append((idx(1, 0, i), idx(1, 1, j), idx(1, 0, j)))

        # Side thickness faces at both Y edges.
        for side in (0, 1):
            faces.append((idx(0, side, i), idx(1, side, i), idx(1, side, j)))
            faces.append((idx(0, side, i), idx(1, side, j), idx(0, side, j)))

    # End caps.
    for i in (0, segments):
        faces.append((idx(0, 0, i), idx(0, 1, i), idx(1, 1, i)))
        faces.append((idx(0, 0, i), idx(1, 1, i), idx(1, 0, i)))

    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_folding_commuter_scooter")

    graphite = model.material("graphite_powder_coat", rgba=(0.06, 0.07, 0.075, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    tire_black = model.material("soft_tread_black", rgba=(0.005, 0.005, 0.004, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    amber = model.material("amber_side_reflector", rgba=(1.0, 0.42, 0.04, 1.0))
    red = model.material("red_tail_lens", rgba=(0.75, 0.02, 0.02, 1.0))

    deck = model.part("deck")
    deck_shell = ExtrudeGeometry(
        rounded_rect_profile(0.70, 0.205, 0.055, corner_segments=10),
        0.052,
        center=True,
    )
    deck.visual(
        mesh_from_geometry(deck_shell, "wide_rounded_deck"),
        origin=Origin(xyz=(-0.035, 0.0, 0.145)),
        material=graphite,
        name="deck_shell",
    )
    footpad = ExtrudeGeometry(
        rounded_rect_profile(0.58, 0.150, 0.035, corner_segments=8),
        0.007,
        center=True,
    )
    deck.visual(
        mesh_from_geometry(footpad, "deck_rubber_footpad"),
        origin=Origin(xyz=(-0.065, 0.0, 0.1745)),
        material=black,
        name="footpad",
    )
    deck.visual(
        Box((0.115, 0.160, 0.060)),
        origin=Origin(xyz=(0.285, 0.0, 0.176)),
        material=graphite,
        name="neck_housing",
    )
    deck.visual(
        Box((0.105, 0.025, 0.028)),
        origin=Origin(xyz=(0.310, -0.075, 0.220)),
        material=dark_metal,
        name="fold_hinge_plate_0",
    )
    deck.visual(
        Box((0.105, 0.025, 0.028)),
        origin=Origin(xyz=(0.310, 0.075, 0.220)),
        material=dark_metal,
        name="fold_hinge_plate_1",
    )

    # Exposed side knuckles at the folding hinge; the moving center knuckle is on the stem.
    hinge_barrel = Cylinder(radius=0.023, length=0.034)
    for side, y in enumerate((-0.056, 0.056)):
        deck.visual(
            hinge_barrel,
            origin=Origin(xyz=(0.300, y, 0.229), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"fold_hinge_knuckle_{side}",
        )

    # Front fork/dropouts sit outside the tire width and visibly support the motor wheel.
    deck.visual(
        Box((0.180, 0.018, 0.030)),
        origin=Origin(xyz=(0.385, -0.058, 0.197)),
        material=dark_metal,
        name="front_fork_crown_0",
    )
    deck.visual(
        Box((0.180, 0.018, 0.030)),
        origin=Origin(xyz=(0.385, 0.058, 0.197)),
        material=dark_metal,
        name="front_fork_crown_1",
    )
    deck.visual(
        Box((0.036, 0.018, 0.175)),
        origin=Origin(xyz=(0.465, -0.058, 0.125)),
        material=dark_metal,
        name="front_fork_leg_0",
    )
    deck.visual(
        Box((0.036, 0.018, 0.175)),
        origin=Origin(xyz=(0.465, 0.058, 0.125)),
        material=dark_metal,
        name="front_fork_leg_1",
    )
    deck.visual(
        Box((0.250, 0.017, 0.032)),
        origin=Origin(xyz=(-0.475, -0.058, 0.112)),
        material=dark_metal,
        name="rear_dropout_arm_0",
    )
    deck.visual(
        Box((0.250, 0.017, 0.032)),
        origin=Origin(xyz=(-0.475, 0.058, 0.112)),
        material=dark_metal,
        name="rear_dropout_arm_1",
    )
    deck.visual(
        Box((0.080, 0.014, 0.070)),
        origin=Origin(xyz=(-0.395, -0.058, 0.199)),
        material=dark_metal,
        name="fender_strut_0",
    )
    deck.visual(
        Box((0.080, 0.014, 0.070)),
        origin=Origin(xyz=(-0.395, 0.058, 0.199)),
        material=dark_metal,
        name="fender_strut_1",
    )

    deck.visual(
        mesh_from_geometry(
            _arched_fender_geometry(
                center_x=-0.505,
                center_z=0.105,
                inner_radius=0.112,
                thickness=0.012,
                width=0.112,
                start_deg=42.0,
                end_deg=162.0,
            ),
            "rear_curved_fender",
        ),
        material=graphite,
        name="rear_fender",
    )
    deck.visual(
        Box((0.050, 0.095, 0.018)),
        origin=Origin(xyz=(-0.647, 0.0, 0.150)),
        material=red,
        name="tail_light",
    )
    for y in (-0.110, 0.110):
        deck.visual(
            Box((0.055, 0.006, 0.018)),
            origin=Origin(xyz=(0.040, y * 0.950, 0.158)),
            material=amber,
            name=f"side_reflector_{'a' if y < 0.0 else 'b'}",
        )

    # Brake-pad hinge knuckles attached to the fixed fender/strut assembly.
    for side, y in enumerate((-0.044, 0.044)):
        deck.visual(
            Cylinder(radius=0.008, length=0.026),
            origin=Origin(xyz=(-0.410, y, 0.232), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"brake_hinge_knuckle_{side}",
        )

    wheel_rim = WheelRim(inner_radius=0.055, flange_height=0.006, flange_thickness=0.004)
    rear_wheel_body = mesh_from_geometry(
        WheelGeometry(
            0.076,
            0.050,
            rim=wheel_rim,
            hub=WheelHub(radius=0.030, width=0.040, cap_style="domed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.003, window_radius=0.009),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "rear_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.105,
            0.062,
            inner_radius=0.075,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.004, count=26, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "commuter_scooter_tire",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_black,
        name="rear_tire",
    )
    rear_wheel.visual(
        rear_wheel_body,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="rear_rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.008, length=0.103),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle_sleeve",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_black,
        name="front_tire",
    )
    front_wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.082,
                0.056,
                rim=WheelRim(inner_radius=0.058, flange_height=0.004, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.050,
                    width=0.052,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.060, hole_diameter=0.004),
                ),
                face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
                spokes=WheelSpokes(style="solid", count=6, thickness=0.004, window_radius=0.006),
                bore=WheelBore(style="round", diameter=0.012),
            ),
            "front_motor_wheel",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=aluminum,
        name="motor_hub",
    )
    front_wheel.visual(
        Cylinder(radius=0.055, length=0.016),
        origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="motor_side_cover",
    )
    front_wheel.visual(
        Cylinder(radius=0.008, length=0.098),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_axle_sleeve",
    )

    lower_stem = model.part("lower_stem")
    lower_stem.visual(
        Cylinder(radius=0.022, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="fold_hinge_knuckle",
    )
    lower_stem.visual(
        Cylinder(radius=0.006, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="fold_hinge_pin",
    )
    lower_stem.visual(
        mesh_from_geometry(_tube_geometry(0.029, 0.019, 0.500, z0=0.020), "lower_stem_sleeve"),
        material=aluminum,
        name="lower_sleeve",
    )
    lower_stem.visual(
        mesh_from_geometry(_tube_geometry(0.038, 0.019, 0.035, z0=0.480), "lower_stem_clamp_collar"),
        material=dark_metal,
        name="clamp_collar",
    )
    lower_stem.visual(
        Box((0.050, 0.018, 0.036)),
        origin=Origin(xyz=(0.063, 0.0, 0.498)),
        material=dark_metal,
        name="quick_release_clamp",
    )
    lower_stem.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(xyz=(0.094, 0.0, 0.498), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="clamp_lever",
    )

    upper_stem = model.part("upper_stem")
    upper_stem.visual(
        Cylinder(radius=0.019, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="inner_tube",
    )
    upper_stem.visual(
        Cylinder(radius=0.018, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.382), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar_bar",
    )
    for side, y in enumerate((-0.225, 0.225)):
        upper_stem.visual(
            Cylinder(radius=0.021, length=0.100),
            origin=Origin(xyz=(0.0, y, 0.382), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"grip_{side}",
        )
    upper_stem.visual(
        Box((0.050, 0.080, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.404)),
        material=black,
        name="display_pod",
    )

    brake_pad = model.part("brake_pad")
    brake_pad.visual(
        Cylinder(radius=0.007, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="brake_hinge_pin",
    )
    brake_pad.visual(
        Box((0.155, 0.084, 0.013)),
        origin=Origin(xyz=(-0.083, 0.0, 0.014)),
        material=black,
        name="rubber_brake_pad",
    )
    brake_pad.visual(
        Box((0.030, 0.076, 0.010)),
        origin=Origin(xyz=(-0.014, 0.0, 0.008)),
        material=dark_metal,
        name="brake_pad_backing",
    )

    model.articulation(
        "deck_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_wheel,
        origin=Origin(xyz=(0.465, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=30.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.505, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "deck_to_lower_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lower_stem,
        origin=Origin(xyz=(0.300, 0.0, 0.229)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.55),
    )
    model.articulation(
        "lower_stem_to_upper_stem",
        ArticulationType.PRISMATIC,
        parent=lower_stem,
        child=upper_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.200),
    )
    model.articulation(
        "deck_to_brake_pad",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=brake_pad,
        origin=Origin(xyz=(-0.410, 0.0, 0.232)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=0.0, upper=0.24),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    lower_stem = object_model.get_part("lower_stem")
    upper_stem = object_model.get_part("upper_stem")
    brake_pad = object_model.get_part("brake_pad")

    fold = object_model.get_articulation("deck_to_lower_stem")
    telescope = object_model.get_articulation("lower_stem_to_upper_stem")
    front_spin = object_model.get_articulation("deck_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")
    brake = object_model.get_articulation("deck_to_brake_pad")

    ctx.allow_overlap(
        lower_stem,
        upper_stem,
        elem_a="lower_sleeve",
        elem_b="inner_tube",
        reason="The telescoping inner tube is intentionally represented as a retained sliding member inside the lower sleeve proxy.",
    )
    ctx.allow_overlap(
        lower_stem,
        upper_stem,
        elem_a="clamp_collar",
        elem_b="inner_tube",
        reason="The clamp collar surrounds the telescoping tube as a simplified captured sleeve.",
    )

    ctx.check(
        "requested articulations are present",
        fold.articulation_type == ArticulationType.REVOLUTE
        and telescope.articulation_type == ArticulationType.PRISMATIC
        and front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and brake.articulation_type == ArticulationType.REVOLUTE,
        details="Scooter should include fold hinge, telescoping stem, rotating wheels, and rear foot brake.",
    )

    ctx.expect_within(
        upper_stem,
        lower_stem,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_sleeve",
        margin=0.0,
        name="telescoping tube centered in lower sleeve",
    )
    ctx.expect_overlap(
        upper_stem,
        lower_stem,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_sleeve",
        min_overlap=0.14,
        name="collapsed telescope keeps long insertion",
    )
    ctx.expect_overlap(
        upper_stem,
        lower_stem,
        axes="z",
        elem_a="inner_tube",
        elem_b="clamp_collar",
        min_overlap=0.030,
        name="clamp collar surrounds the telescoping tube",
    )

    with ctx.pose({telescope: 0.200}):
        ctx.expect_within(
            upper_stem,
            lower_stem,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_sleeve",
            margin=0.0,
            name="extended telescope remains centered",
        )
        ctx.expect_overlap(
            upper_stem,
            lower_stem,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_sleeve",
            min_overlap=0.14,
            name="extended telescope retains insertion",
        )

    rest_stem_aabb = ctx.part_element_world_aabb(lower_stem, elem="lower_sleeve")
    with ctx.pose({fold: 1.20}):
        folded_stem_aabb = ctx.part_element_world_aabb(lower_stem, elem="lower_sleeve")
    ctx.check(
        "fold hinge swings stem back toward deck",
        rest_stem_aabb is not None
        and folded_stem_aabb is not None
        and folded_stem_aabb[0][0] < rest_stem_aabb[0][0] - 0.25
        and folded_stem_aabb[1][2] < rest_stem_aabb[1][2] - 0.10,
        details=f"rest={rest_stem_aabb}, folded={folded_stem_aabb}",
    )

    # The tires are retained between fork/dropout members with side clearance.
    ctx.expect_overlap(front_wheel, deck, axes="xz", elem_a="front_tire", elem_b="front_fork_leg_0", min_overlap=0.020)
    ctx.expect_gap(deck, front_wheel, axis="y", positive_elem="front_fork_leg_1", negative_elem="front_tire", min_gap=0.010)
    ctx.expect_overlap(rear_wheel, deck, axes="xz", elem_a="rear_tire", elem_b="rear_dropout_arm_0", min_overlap=0.020)

    rest_pad_aabb = ctx.part_element_world_aabb(brake_pad, elem="rubber_brake_pad")
    with ctx.pose({brake: 0.20}):
        depressed_pad_aabb = ctx.part_element_world_aabb(brake_pad, elem="rubber_brake_pad")
    ctx.check(
        "rear brake pad depresses toward the fender",
        rest_pad_aabb is not None
        and depressed_pad_aabb is not None
        and depressed_pad_aabb[0][2] < rest_pad_aabb[0][2] - 0.010,
        details=f"rest={rest_pad_aabb}, depressed={depressed_pad_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modular_yard_ramp")

    steel = model.material("galvanized_steel", rgba=(0.46, 0.50, 0.50, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    deck_length = 2.40
    deck_width = 1.18
    deck_thickness = 0.070
    front_ground_clearance = 0.105
    incline = math.radians(15.0)
    c = math.cos(incline)
    s = math.sin(incline)
    deck_rpy = (0.0, -incline, 0.0)

    def deck_xyz(along: float, across: float = 0.0, normal: float = 0.0) -> tuple[float, float, float]:
        """World location of a point expressed in deck local coordinates."""
        return (
            c * along - s * normal,
            across,
            front_ground_clearance + s * along + c * normal,
        )

    def deck_visual(
        part,
        name: str,
        size: tuple[float, float, float],
        along: float,
        across: float,
        normal: float,
        material: Material,
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=deck_xyz(along, across, normal), rpy=deck_rpy),
            material=material,
            name=name,
        )

    deck = model.part("deck")
    deck_visual(
        deck,
        "deck_plate",
        (deck_length, deck_width, deck_thickness),
        deck_length * 0.5,
        0.0,
        0.0,
        steel,
    )

    # Raised transverse anti-slip ribs welded to the tread plate.
    rib_count = 18
    rib_height = 0.020
    for i in range(rib_count):
        along = 0.16 + i * (deck_length - 0.32) / (rib_count - 1)
        deck_visual(
            deck,
            f"rib_{i:02d}",
            (0.030, deck_width - 0.10, rib_height),
            along,
            0.0,
            deck_thickness * 0.5 + rib_height * 0.5 - 0.003,
            dark_steel,
        )

    # Long side channel stringers and end lips make the module read as a steel ramp section.
    for side_index, across in enumerate((-deck_width * 0.5 - 0.030, deck_width * 0.5 + 0.030)):
        deck_visual(
            deck,
            f"side_channel_{side_index}",
            (deck_length, 0.060, 0.125),
            deck_length * 0.5,
            across,
            -0.045,
            dark_steel,
        )
    deck_visual(deck, "front_lip", (0.090, deck_width + 0.11, 0.095), 0.030, 0.0, -0.020, dark_steel)
    deck_visual(deck, "rear_stop", (0.080, deck_width + 0.12, 0.165), deck_length - 0.035, 0.0, 0.020, dark_steel)

    # Underside hinge clevis plates for the two folding leg pairs.
    leg_hinge_stations = (0.70, 1.55)
    for station_index, along in enumerate(leg_hinge_stations):
        for side_index, across in enumerate((-0.535, 0.535)):
            deck_visual(
                deck,
                f"leg_clevis_{station_index}_{side_index}",
                (0.085, 0.036, 0.125),
                along,
                across,
                -deck_thickness * 0.5 - 0.052,
                dark_steel,
            )

    # Rear guard hinge pads sit along the two long upper edges.
    guard_start = 1.30
    guard_length = 0.96
    for side_index, side in enumerate((-1.0, 1.0)):
        across = side * (deck_width * 0.5 + 0.028)
        for along in (guard_start + 0.10, guard_start + guard_length - 0.10):
            deck_visual(
                deck,
                f"guard_hinge_pad_{side_index}_{int((along - guard_start) * 100):02d}",
                (0.155, 0.045, 0.040),
                along,
                across,
                deck_thickness * 0.5 - 0.002,
                dark_steel,
            )

    def add_transverse_cylinder(part, name: str, radius: float, length: float, xyz, material: Material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def add_longitudinal_cylinder(part, name: str, radius: float, length: float, xyz, material: Material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0 - incline, 0.0)),
            material=material,
            name=name,
        )

    # Two hinged folding support leg pairs.  Each pair is a one-piece welded U-frame
    # with a transverse hinge sleeve, two box-tube legs, and a rubber-footed spreader bar.
    for pair_index, along in enumerate(leg_hinge_stations):
        hinge_xyz = deck_xyz(along, 0.0, -deck_thickness * 0.5 - 0.078)
        leg_length = hinge_xyz[2] - 0.032

        pair = model.part(f"support_pair_{pair_index}")
        pair.visual(
            Cylinder(radius=0.030, length=1.036),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="hinge_tube",
        )
        for side_index, across in enumerate((-0.410, 0.410)):
            pair.visual(
                Box((0.060, 0.060, leg_length)),
                origin=Origin(xyz=(0.0, across, -leg_length * 0.5)),
                material=dark_steel,
                name=f"leg_{side_index}",
            )
            pair.visual(
                Box((0.150, 0.090, 0.032)),
                origin=Origin(xyz=(0.020, across, -leg_length - 0.016)),
                material=rubber,
                name=f"foot_pad_{side_index}",
            )
        add_transverse_cylinder(pair, "foot_bar", 0.024, 0.940, (0.020, 0.0, -leg_length), dark_steel)
        pair.visual(
            Box((0.060, 0.900, 0.038)),
            origin=Origin(xyz=(0.020, 0.0, -leg_length + 0.005)),
            material=dark_steel,
            name="lower_tie",
        )

        model.articulation(
            f"deck_to_support_pair_{pair_index}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=pair,
            origin=Origin(xyz=hinge_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=450.0, velocity=1.0, lower=0.0, upper=1.35),
        )

    # Two folding rear guard railings, hinged on the long edges of the raised rear deck.
    rail_height = 0.66
    for guard_index, side in enumerate((-1.0, 1.0)):
        hinge_xyz = deck_xyz(guard_start, side * (deck_width * 0.5 + 0.0735), deck_thickness * 0.5 + 0.035)
        guard = model.part(f"rear_guard_{guard_index}")

        add_longitudinal_cylinder(guard, "hinge_tube", 0.023, guard_length, (c * guard_length * 0.5, 0.0, s * guard_length * 0.5), dark_steel)
        guard.visual(
            Box((0.145, 0.046, 0.040)),
            origin=Origin(xyz=(c * 0.10, 0.0, s * 0.10), rpy=deck_rpy),
            material=dark_steel,
            name="hinge_leaf_0",
        )
        guard.visual(
            Box((0.145, 0.046, 0.040)),
            origin=Origin(xyz=(c * (guard_length - 0.10), 0.0, s * (guard_length - 0.10)), rpy=deck_rpy),
            material=dark_steel,
            name="hinge_leaf_1",
        )
        for post_index, dist in enumerate((0.07, guard_length * 0.50, guard_length - 0.07)):
            guard.visual(
                Cylinder(radius=0.022, length=rail_height),
                origin=Origin(xyz=(c * dist, 0.0, s * dist + rail_height * 0.5)),
                material=safety_yellow,
                name=f"post_{post_index}",
            )
        add_longitudinal_cylinder(
            guard,
            "mid_rail",
            0.019,
            guard_length - 0.08,
            (c * guard_length * 0.5, 0.0, s * guard_length * 0.5 + rail_height * 0.42),
            safety_yellow,
        )
        add_longitudinal_cylinder(
            guard,
            "top_rail",
            0.024,
            guard_length,
            (c * guard_length * 0.5, 0.0, s * guard_length * 0.5 + rail_height),
            safety_yellow,
        )
        # Small diagonal brace from the hinge sleeve to the center post stiffens the guard frame.
        brace_len = math.sqrt((c * guard_length * 0.38) ** 2 + (s * guard_length * 0.38 + rail_height * 0.42) ** 2)
        brace_pitch = math.atan2(c * guard_length * 0.38, s * guard_length * 0.38 + rail_height * 0.42)
        guard.visual(
            Cylinder(radius=0.014, length=brace_len),
            origin=Origin(
                xyz=(c * guard_length * 0.19, 0.0, s * guard_length * 0.19 + rail_height * 0.21),
                rpy=(0.0, brace_pitch, 0.0),
            ),
            material=safety_yellow,
            name="diagonal_brace",
        )

        # Flip the axis on the positive-Y side so positive motion folds each guard outward.
        rail_axis = (side * c, 0.0, side * s)
        model.articulation(
            f"deck_to_rear_guard_{guard_index}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=guard,
            origin=Origin(xyz=hinge_xyz),
            axis=rail_axis,
            motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=math.radians(92.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def coord(vec, index: int) -> float:
        if hasattr(vec, "__getitem__"):
            return vec[index]
        return (vec.x, vec.y, vec.z)[index]

    def aabb_center(bounds, index: int) -> float:
        return 0.5 * (coord(bounds[0], index) + coord(bounds[1], index))

    deck = object_model.get_part("deck")
    support_pairs = [object_model.get_part(f"support_pair_{i}") for i in range(2)]
    rear_guards = [object_model.get_part(f"rear_guard_{i}") for i in range(2)]

    support_joints = [object_model.get_articulation(f"deck_to_support_pair_{i}") for i in range(2)]
    guard_joints = [object_model.get_articulation(f"deck_to_rear_guard_{i}") for i in range(2)]
    all_hinges = support_joints + guard_joints

    ctx.check(
        "four deck-mounted revolute hinges",
        len(all_hinges) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in all_hinges),
        details=f"hinges={[j.name for j in all_hinges]}",
    )
    ctx.check(
        "hinges have folding travel",
        all(j.motion_limits is not None and j.motion_limits.lower == 0.0 and j.motion_limits.upper and j.motion_limits.upper > 1.0 for j in all_hinges),
        details="Each support pair and rear guard should fold through a realistic revolute range.",
    )

    rib_names = [v.name for v in deck.visuals if v.name and v.name.startswith("rib_")]
    ctx.check("deck has repeated anti-slip ribs", len(rib_names) >= 16, details=f"rib_count={len(rib_names)}")

    deck_bounds = ctx.part_element_world_aabb(deck, elem="deck_plate")
    ctx.check(
        "deck is visibly inclined",
        deck_bounds is not None
        and coord(deck_bounds[1], 2) - coord(deck_bounds[0], 2) > 0.55
        and coord(deck_bounds[1], 0) - coord(deck_bounds[0], 0) > 2.2,
        details=f"deck_bounds={deck_bounds}",
    )

    # The transverse support hinge tubes are captured in deck clevis plates.  A
    # tiny tube-end embed keeps the pin visibly seated and prevents a floating
    # hinge reading; it is local to the pin/lug interface.
    for pair_index, pair in enumerate(support_pairs):
        for clevis_index in range(2):
            clevis_name = f"leg_clevis_{pair_index}_{clevis_index}"
            ctx.allow_overlap(
                deck,
                pair,
                elem_a=clevis_name,
                elem_b="hinge_tube",
                reason="The folding support hinge pin is intentionally captured in the underside clevis lug.",
            )
            ctx.expect_contact(
                deck,
                pair,
                elem_a=clevis_name,
                elem_b="hinge_tube",
                contact_tol=0.002,
                name=f"support pair {pair_index} hinge pin seats in clevis {clevis_index}",
            )
            ctx.expect_overlap(
                deck,
                pair,
                axes="xz",
                elem_a=clevis_name,
                elem_b="hinge_tube",
                min_overlap=0.025,
                name=f"support pair {pair_index} clevis surrounds pin {clevis_index}",
            )

        foot_bounds = ctx.part_element_world_aabb(pair, elem="foot_pad_0")
        ctx.check(
            f"support pair {pair_index} foot reaches ground",
            foot_bounds is not None and abs(coord(foot_bounds[0], 2)) < 0.006,
            details=f"foot_bounds={foot_bounds}",
        )

        rest_bar = ctx.part_element_world_aabb(pair, elem="foot_bar")
        with ctx.pose({support_joints[pair_index]: 1.2}):
            folded_bar = ctx.part_element_world_aabb(pair, elem="foot_bar")
        ctx.check(
            f"support pair {pair_index} folds upward",
            rest_bar is not None
            and folded_bar is not None
            and coord(folded_bar[0], 2) > coord(rest_bar[0], 2) + 0.045,
            details=f"rest={rest_bar}, folded={folded_bar}",
        )

    for guard_index, guard in enumerate(rear_guards):
        pad_name = f"guard_hinge_pad_{guard_index}_85"
        ctx.expect_contact(
            deck,
            guard,
            elem_a=pad_name,
            elem_b="hinge_leaf_1",
            contact_tol=0.001,
            name=f"rear guard {guard_index} hinge leaf sits on long-edge pad",
        )

        rest_top = ctx.part_element_world_aabb(guard, elem="top_rail")
        with ctx.pose({guard_joints[guard_index]: 1.2}):
            folded_top = ctx.part_element_world_aabb(guard, elem="top_rail")
        ctx.check(
            f"rear guard {guard_index} folds down from upright",
            rest_top is not None
            and folded_top is not None
            and coord(folded_top[1], 2) < coord(rest_top[1], 2) - 0.25
            and abs(aabb_center(folded_top, 1)) < abs(aabb_center(rest_top, 1)) - 0.15,
            details=f"rest={rest_top}, folded={folded_top}",
        )

    return ctx.report()


object_model = build_object_model()

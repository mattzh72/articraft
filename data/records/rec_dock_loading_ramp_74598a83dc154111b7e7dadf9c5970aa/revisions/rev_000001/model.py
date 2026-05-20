from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_truck_bed_ramp")

    steel_dark = model.material("steel_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    steel_plate = model.material("steel_plate", rgba=(0.39, 0.41, 0.43, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.87, 0.9, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    deck_length = 3.20
    deck_width = 2.28
    deck_thickness = 0.02
    deck_pitch = math.radians(12.0)
    deck_center = (0.0, 0.0, 0.88)

    approach_length = 0.55
    approach_width = 2.10
    approach_thickness = deck_thickness

    leg_mount_x = 0.55
    leg_mount_y = 0.60
    housing_flange_size = (0.24, 0.18, 0.016)
    housing_body_radius = 0.075
    housing_body_length = 0.32
    brace_size = (0.18, 0.02, 0.18)
    leg_head_radius = 0.055
    leg_head_length = 0.032
    leg_rod_radius = 0.045
    leg_rod_length = 0.42
    foot_size = (0.28, 0.18, 0.03)

    def rotate_y(
        point: tuple[float, float, float], angle: float
    ) -> tuple[float, float, float]:
        x, y, z = point
        c = math.cos(angle)
        s = math.sin(angle)
        return (c * x + s * z, y, -s * x + c * z)

    def add(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

    def deck_origin(local_center: tuple[float, float, float]) -> Origin:
        return Origin(
            xyz=add(deck_center, rotate_y(local_center, deck_pitch)),
            rpy=(0.0, deck_pitch, 0.0),
        )

    def deck_point(local_point: tuple[float, float, float]) -> tuple[float, float, float]:
        return add(deck_center, rotate_y(local_point, deck_pitch))

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=deck_origin((0.0, 0.0, 0.0)),
        material=steel_plate,
        name="deck_slab",
    )

    rail_width = 0.08
    rail_height = 0.10
    for side_name, side_y in (("left", leg_mount_y + 0.46), ("right", -(leg_mount_y + 0.46))):
        deck.visual(
            Box((deck_length - 0.08, rail_width, rail_height)),
            origin=deck_origin(
                (
                    -0.02,
                    side_y,
                    deck_thickness / 2.0 + rail_height / 2.0,
                )
            ),
            material=steel_dark,
            name=f"{side_name}_side_rail",
        )

    headboard_width = 0.10
    headboard_height = 0.16
    deck.visual(
        Box((headboard_width, deck_width, headboard_height)),
        origin=deck_origin(
            (
                -deck_length / 2.0 + headboard_width / 2.0,
                0.0,
                deck_thickness / 2.0 + headboard_height / 2.0,
            )
        ),
        material=steel_dark,
        name="rear_headboard",
    )

    rib_height = 0.012
    rib_length = 0.085
    rib_span = approach_width - 0.10
    for idx in range(11):
        x_pos = -1.15 + idx * 0.23
        deck.visual(
            Box((rib_length, rib_span, rib_height)),
            origin=deck_origin((x_pos, 0.0, deck_thickness / 2.0 + rib_height / 2.0)),
            material=steel_dark,
            name=f"deck_rib_{idx + 1}",
        )

    stringer_height = 0.22
    stringer_width = 0.15
    stringer_length = 2.86
    for side_name, side_y in (("left", 0.82), ("right", -0.82)):
        deck.visual(
            Box((stringer_length, stringer_width, stringer_height)),
            origin=deck_origin(
                (
                    -0.04,
                    side_y,
                    -deck_thickness / 2.0 - stringer_height / 2.0,
                )
            ),
            material=steel_dark,
            name=f"{side_name}_stringer",
        )

    for idx, x_pos in enumerate((-0.85, 0.05, 0.95)):
        deck.visual(
            Box((0.12, 1.80, 0.08)),
            origin=deck_origin(
                (
                    x_pos,
                    0.0,
                    -deck_thickness / 2.0 - 0.04,
                )
            ),
            material=steel_dark,
            name=f"crossmember_{idx + 1}",
        )

    deck.inertial = Inertial.from_geometry(
        Box((deck_length, deck_width, 0.36)),
        mass=820.0,
        origin=Origin(xyz=(deck_center[0], deck_center[1], deck_center[2] - 0.10)),
    )

    def add_leg_housing(name: str) -> object:
        housing = model.part(name)
        housing.visual(
            Box(housing_flange_size),
            origin=Origin(xyz=(0.0, 0.0, -housing_flange_size[2] / 2.0)),
            material=steel_dark,
            name="mount_flange",
        )
        housing.visual(
            Cylinder(radius=housing_body_radius, length=housing_body_length),
            origin=Origin(
                xyz=(0.0, 0.0, -housing_flange_size[2] - housing_body_length / 2.0)
            ),
            material=steel_dark,
            name="outer_barrel",
        )
        for brace_name, brace_y in (("left_brace", 0.08), ("right_brace", -0.08)):
            housing.visual(
                Box(brace_size),
                origin=Origin(
                    xyz=(
                        0.0,
                        brace_y,
                        -housing_flange_size[2] - brace_size[2] / 2.0,
                    )
                ),
                material=steel_dark,
                name=brace_name,
            )

        housing.inertial = Inertial.from_geometry(
            Box((housing_flange_size[0], housing_flange_size[1], housing_flange_size[2] + housing_body_length)),
            mass=70.0,
            origin=Origin(
                xyz=(0.0, 0.0, -(housing_flange_size[2] + housing_body_length) / 2.0)
            ),
        )
        return housing

    def add_leg_post(name: str) -> object:
        leg = model.part(name)
        leg.visual(
            Cylinder(radius=leg_head_radius, length=leg_head_length),
            origin=Origin(xyz=(0.0, 0.0, -leg_head_length / 2.0)),
            material=steel_dark,
            name="rod_head",
        )
        leg.visual(
            Cylinder(radius=leg_rod_radius, length=leg_rod_length),
            origin=Origin(xyz=(0.0, 0.0, -leg_head_length - leg_rod_length / 2.0)),
            material=chrome,
            name="hydraulic_ram",
        )
        leg.visual(
            Box(foot_size),
            origin=Origin(
                xyz=(0.0, 0.0, -leg_head_length - leg_rod_length - foot_size[2] / 2.0)
            ),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Cylinder(radius=0.07, length=leg_rod_length + foot_size[2] + leg_head_length),
            mass=46.0,
            origin=Origin(
                xyz=(0.0, 0.0, -(leg_head_length + leg_rod_length + foot_size[2]) / 2.0)
            ),
        )
        return leg

    left_leg_housing = add_leg_housing("left_leg_housing")
    right_leg_housing = add_leg_housing("right_leg_housing")
    left_leg = add_leg_post("left_leg")
    right_leg = add_leg_post("right_leg")

    for side_name, side_sign, housing in (
        ("left", 1.0, left_leg_housing),
        ("right", -1.0, right_leg_housing),
    ):
        model.articulation(
            f"deck_to_{side_name}_leg_housing",
            ArticulationType.FIXED,
            parent=deck,
            child=housing,
            origin=Origin(
                xyz=deck_point((leg_mount_x, side_sign * leg_mount_y, -deck_thickness / 2.0)),
                rpy=(0.0, deck_pitch, 0.0),
            ),
        )

    for side_name, housing, leg in (
        ("left", left_leg_housing, left_leg),
        ("right", right_leg_housing, right_leg),
    ):
        model.articulation(
            f"{side_name}_leg_slide",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=leg,
            origin=Origin(xyz=(0.0, 0.0, -housing_flange_size[2] - housing_body_length)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=22000.0,
                velocity=0.18,
                lower=0.0,
                upper=0.26,
            ),
        )

    approach_plate = model.part("approach_plate")
    approach_plate.visual(
        Box((approach_length, approach_width, approach_thickness)),
        origin=Origin(
            xyz=rotate_y((approach_length / 2.0, 0.0, 0.0), deck_pitch),
            rpy=(0.0, deck_pitch, 0.0),
        ),
        material=steel_plate,
        name="plate_shell",
    )

    plate_rib_height = 0.014
    plate_rib_span = approach_width - 0.10
    for idx in range(4):
        plate_x = 0.11 + idx * 0.10
        approach_plate.visual(
            Box((0.06, plate_rib_span, plate_rib_height)),
            origin=Origin(
                xyz=rotate_y(
                    (
                        plate_x,
                        0.0,
                        approach_thickness / 2.0 + plate_rib_height / 2.0,
                    ),
                    deck_pitch,
                ),
                rpy=(0.0, deck_pitch, 0.0),
            ),
            material=steel_dark,
            name=f"plate_rib_{idx + 1}",
        )

    approach_plate.visual(
        Box((0.08, approach_width, 0.02)),
        origin=Origin(
            xyz=rotate_y((approach_length - 0.04, 0.0, -approach_thickness / 2.0 - 0.01), deck_pitch),
            rpy=(0.0, deck_pitch + math.radians(18.0), 0.0),
        ),
        material=steel_dark,
        name="lead_edge",
    )
    approach_plate.inertial = Inertial.from_geometry(
        Box((approach_length, approach_width, approach_thickness)),
        mass=95.0,
        origin=Origin(
            xyz=rotate_y((approach_length / 2.0, 0.0, 0.0), deck_pitch),
            rpy=(0.0, deck_pitch, 0.0),
        ),
    )

    model.articulation(
        "deck_to_approach_plate",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=approach_plate,
        origin=Origin(xyz=deck_point((deck_length / 2.0, 0.0, 0.0))),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5000.0,
            velocity=1.0,
            lower=-0.12,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    deck = object_model.get_part("deck")
    left_leg_housing = object_model.get_part("left_leg_housing")
    right_leg_housing = object_model.get_part("right_leg_housing")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    approach_plate = object_model.get_part("approach_plate")

    left_leg_slide = object_model.get_articulation("left_leg_slide")
    right_leg_slide = object_model.get_articulation("right_leg_slide")
    plate_hinge = object_model.get_articulation("deck_to_approach_plate")

    ctx.expect_contact(left_leg_housing, deck, name="left housing welded to deck")
    ctx.expect_contact(right_leg_housing, deck, name="right housing welded to deck")
    ctx.expect_contact(left_leg, left_leg_housing, name="left telescoping leg seated in housing")
    ctx.expect_contact(right_leg, right_leg_housing, name="right telescoping leg seated in housing")
    ctx.expect_contact(approach_plate, deck, name="approach plate meets front deck edge")
    ctx.expect_overlap(
        approach_plate,
        deck,
        axes="y",
        min_overlap=1.95,
        name="approach plate spans the usable deck width",
    )
    ctx.expect_origin_distance(
        left_leg_housing,
        right_leg_housing,
        axes="y",
        min_dist=1.15,
        max_dist=1.25,
        name="leg housings are spaced across the deck",
    )

    left_rest = ctx.part_world_position(left_leg)
    right_rest = ctx.part_world_position(right_leg)
    if left_rest is None or right_rest is None:
        ctx.fail("leg positions available", "Could not resolve leg world positions in rest pose.")
        return ctx.report()

    extension_amount = 0.22

    with ctx.pose({left_leg_slide: extension_amount, right_leg_slide: extension_amount}):
        left_extended = ctx.part_world_position(left_leg)
        right_extended = ctx.part_world_position(right_leg)
        if left_extended is None or right_extended is None:
            ctx.fail(
                "leg positions available in extended pose",
                "Could not resolve leg world positions after extending the prismatic joints.",
            )
        else:
            left_delta = (
                left_extended[0] - left_rest[0],
                left_extended[1] - left_rest[1],
                left_extended[2] - left_rest[2],
            )
            right_delta = (
                right_extended[0] - right_rest[0],
                right_extended[1] - right_rest[1],
                right_extended[2] - right_rest[2],
            )
            ctx.check(
                "left leg extends downward along the inclined post",
                -0.055 <= left_delta[0] <= -0.035 and abs(left_delta[1]) <= 0.002 and left_delta[2] < -0.20,
                details=(
                    f"left leg delta={left_delta}, "
                    f"axis={left_leg_slide.axis}"
                ),
            )
            ctx.check(
                "right leg extends downward along the inclined post",
                -0.055 <= right_delta[0] <= -0.035 and abs(right_delta[1]) <= 0.002 and right_delta[2] < -0.20,
                details=(
                    f"right leg delta={right_delta}, "
                    f"axis={right_leg_slide.axis}"
                ),
            )

    plate_rest_aabb = ctx.part_element_world_aabb(approach_plate, elem="plate_shell")
    if plate_rest_aabb is None:
        ctx.fail("approach plate AABB available", "Could not resolve the plate shell AABB in rest pose.")
        return ctx.report()

    with ctx.pose({plate_hinge: 0.65}):
        plate_down_aabb = ctx.part_element_world_aabb(approach_plate, elem="plate_shell")
        if plate_down_aabb is None:
            ctx.fail(
                "approach plate AABB available when folded",
                "Could not resolve the plate shell AABB with the hinge articulated.",
            )
        else:
            ctx.check(
                "approach plate folds downward from the deck edge",
                plate_down_aabb[0][2] < plate_rest_aabb[0][2] - 0.15,
                details=(
                    f"rest min z={plate_rest_aabb[0][2]:.4f}, "
                    f"folded min z={plate_down_aabb[0][2]:.4f}"
                ),
            )
            ctx.expect_overlap(
                approach_plate,
                deck,
                axes="y",
                min_overlap=1.95,
                name="approach plate stays aligned across the hinge line",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_W = 0.43
BODY_D = 0.28
BODY_H = 0.085
WALL_T = 0.004
RIM_T = 0.005
LID_W = 0.446
LID_D = 0.296
LID_SKIRT_H = 0.058
LID_TOP_T = 0.004
LEG_LEN = 0.118
LEG_TILT = math.radians(18.0)
LEG_FOLD = -math.radians(109.0)


def _rr_loop(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _build_lid_top_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(LID_W, LID_D, 0.030, corner_segments=8),
            LID_TOP_T,
        ),
        "lid_top_plate",
    )


def _build_lid_crown_mesh():
    crown = section_loft(
        [
            _rr_loop(0.365, 0.215, 0.026, 0.000),
            _rr_loop(0.338, 0.190, 0.023, 0.014),
            _rr_loop(0.305, 0.162, 0.018, 0.026),
        ]
    )
    return mesh_from_geometry(crown, "lid_crown")


def _add_grate_bars(grate, grate_w: float, bar_y_positions: list[float], z_center: float, material) -> None:
    for index, y_pos in enumerate(bar_y_positions):
        grate.visual(
            Box((grate_w, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, y_pos, z_center)),
            material=material,
            name=f"grate_bar_{index}",
        )


def _build_leg(part, *, forward: bool, material) -> None:
    tilt = LEG_TILT if forward else -LEG_TILT
    part.visual(
        Box((0.022, 0.010, LEG_LEN + 0.012)),
        origin=Origin(xyz=(0.0, 0.010 if forward else -0.010, -0.064), rpy=(tilt, 0.0, 0.0)),
        material=material,
        name="leg_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_tabletop_grill")

    enamel = model.material("enamel", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.30, 1.0))
    grate_metal = model.material("grate_metal", rgba=(0.44, 0.45, 0.46, 1.0))
    leg_metal = model.material("leg_metal", rgba=(0.70, 0.71, 0.73, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.62, 0.63, 0.64, 1.0))
    handle_grip = model.material("handle_grip", rgba=(0.10, 0.10, 0.10, 1.0))

    base = model.part("base_body")
    base.visual(
        Box((BODY_W, BODY_D, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=enamel,
        name="firebox_floor",
    )
    base.visual(
        Box((WALL_T, BODY_D, 0.078)),
        origin=Origin(xyz=(-(BODY_W - WALL_T) / 2.0, 0.0, 0.042)),
        material=enamel,
        name="left_wall",
    )
    base.visual(
        Box((WALL_T, BODY_D, 0.078)),
        origin=Origin(xyz=((BODY_W - WALL_T) / 2.0, 0.0, 0.042)),
        material=enamel,
        name="right_wall",
    )
    base.visual(
        Box((BODY_W - (2.0 * WALL_T), WALL_T, 0.078)),
        origin=Origin(xyz=(0.0, -(BODY_D - WALL_T) / 2.0, 0.042)),
        material=enamel,
        name="rear_wall",
    )
    base.visual(
        Box((BODY_W - (2.0 * WALL_T), WALL_T, 0.078)),
        origin=Origin(xyz=(0.0, (BODY_D - WALL_T) / 2.0, 0.042)),
        material=enamel,
        name="front_wall",
    )
    base.visual(
        Box((BODY_W, 0.016, RIM_T)),
        origin=Origin(xyz=(0.0, -(BODY_D / 2.0) + 0.008, 0.0825)),
        material=enamel,
        name="rear_rim",
    )
    base.visual(
        Box((BODY_W, 0.016, RIM_T)),
        origin=Origin(xyz=(0.0, (BODY_D / 2.0) - 0.008, 0.0825)),
        material=enamel,
        name="front_rim",
    )
    base.visual(
        Box((0.016, BODY_D - 0.028, RIM_T)),
        origin=Origin(xyz=(-(BODY_W / 2.0) + 0.008, 0.0, 0.0825)),
        material=enamel,
        name="left_rim",
    )
    base.visual(
        Box((0.016, BODY_D - 0.028, RIM_T)),
        origin=Origin(xyz=((BODY_W / 2.0) - 0.008, 0.0, 0.0825)),
        material=enamel,
        name="right_rim",
    )
    base.visual(
        Box((0.010, BODY_D - 0.040, 0.004)),
        origin=Origin(xyz=(-(BODY_W / 2.0) + 0.009, 0.0, 0.053)),
        material=dark_metal,
        name="left_grate_ledge",
    )
    base.visual(
        Box((0.010, BODY_D - 0.040, 0.004)),
        origin=Origin(xyz=((BODY_W / 2.0) - 0.009, 0.0, 0.053)),
        material=dark_metal,
        name="right_grate_ledge",
    )

    pivot_x = 0.152
    pivot_y = 0.090
    for name, x_pos, y_pos in (
        ("front_left_bracket", -pivot_x, pivot_y),
        ("front_right_bracket", pivot_x, pivot_y),
        ("rear_left_bracket", -pivot_x, -pivot_y),
        ("rear_right_bracket", pivot_x, -pivot_y),
    ):
        base.visual(
            Box((0.024, 0.010, 0.006)),
            origin=Origin(xyz=(x_pos, y_pos + (0.010 if y_pos > 0.0 else -0.010), 0.003)),
            material=dark_metal,
            name=name,
        )

    base.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    grate = model.part("cook_grate")
    grate_w = 0.402
    grate_d = 0.226
    grate_z = 0.0575
    grate.visual(
        Box((0.010, grate_d, 0.005)),
        origin=Origin(xyz=(-0.206, 0.0, grate_z)),
        material=grate_metal,
        name="grate_left_frame",
    )
    grate.visual(
        Box((0.010, grate_d, 0.005)),
        origin=Origin(xyz=(0.206, 0.0, grate_z)),
        material=grate_metal,
        name="grate_right_frame",
    )
    grate.visual(
        Box((0.422, 0.008, 0.005)),
        origin=Origin(xyz=(0.0, -grate_d / 2.0, grate_z)),
        material=grate_metal,
        name="grate_rear_frame",
    )
    grate.visual(
        Box((0.422, 0.008, 0.005)),
        origin=Origin(xyz=(0.0, grate_d / 2.0, grate_z)),
        material=grate_metal,
        name="grate_front_frame",
    )
    _add_grate_bars(
        grate,
        grate_w,
        [-0.072, -0.043, -0.014, 0.014, 0.043, 0.072],
        grate_z,
        grate_metal,
    )
    grate.inertial = Inertial.from_geometry(
        Box((grate_w + 0.008, grate_d + 0.008, 0.010)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, grate_z)),
    )

    lid = model.part("lid")
    lid.visual(
        _build_lid_top_mesh(),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_SKIRT_H - 0.002)),
        material=enamel,
        name="lid_top_plate",
    )
    lid.visual(
        _build_lid_crown_mesh(),
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_SKIRT_H + 0.001)),
        material=enamel,
        name="lid_crown",
    )
    lid.visual(
        Box((LID_W, WALL_T, LID_SKIRT_H)),
        origin=Origin(xyz=(0.0, WALL_T / 2.0, LID_SKIRT_H / 2.0)),
        material=enamel,
        name="lid_rear_skirt",
    )
    lid.visual(
        Box((LID_W, WALL_T, LID_SKIRT_H)),
        origin=Origin(xyz=(0.0, LID_D - (WALL_T / 2.0), LID_SKIRT_H / 2.0)),
        material=enamel,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((WALL_T, LID_D - (2.0 * WALL_T), LID_SKIRT_H)),
        origin=Origin(xyz=(-(LID_W - WALL_T) / 2.0, LID_D / 2.0, LID_SKIRT_H / 2.0)),
        material=enamel,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((WALL_T, LID_D - (2.0 * WALL_T), LID_SKIRT_H)),
        origin=Origin(xyz=((LID_W - WALL_T) / 2.0, LID_D / 2.0, LID_SKIRT_H / 2.0)),
        material=enamel,
        name="lid_right_skirt",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(-0.062, LID_D - 0.036, 0.065)),
        material=handle_metal,
        name="handle_left_post",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.062, LID_D - 0.036, 0.065)),
        material=handle_metal,
        name="handle_right_post",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.150),
        origin=Origin(
            xyz=(0.0, LID_D - 0.036, 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_grip,
        name="handle_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, 0.095)),
        mass=2.2,
        origin=Origin(xyz=(0.0, LID_D / 2.0, 0.0475)),
    )

    front_left_leg = model.part("front_left_leg")
    _build_leg(front_left_leg, forward=True, material=leg_metal)
    front_left_leg.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, LEG_LEN + 0.010)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    front_right_leg = model.part("front_right_leg")
    _build_leg(front_right_leg, forward=True, material=leg_metal)
    front_right_leg.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, LEG_LEN + 0.010)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    rear_left_leg = model.part("rear_left_leg")
    _build_leg(rear_left_leg, forward=False, material=leg_metal)
    rear_left_leg.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, LEG_LEN + 0.010)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    rear_right_leg = model.part("rear_right_leg")
    _build_leg(rear_right_leg, forward=False, material=leg_metal)
    rear_right_leg.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, LEG_LEN + 0.010)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    model.articulation(
        "base_to_grate",
        ArticulationType.FIXED,
        parent=base,
        child=grate,
        origin=Origin(),
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, -(BODY_D / 2.0), BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "base_to_front_left_leg",
        ArticulationType.REVOLUTE,
        parent=base,
        child=front_left_leg,
        origin=Origin(xyz=(-pivot_x, pivot_y, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=LEG_FOLD,
            upper=0.0,
        ),
    )
    model.articulation(
        "base_to_front_right_leg",
        ArticulationType.REVOLUTE,
        parent=base,
        child=front_right_leg,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=LEG_FOLD,
            upper=0.0,
        ),
    )
    model.articulation(
        "base_to_rear_left_leg",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_left_leg,
        origin=Origin(xyz=(-pivot_x, -pivot_y, 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=LEG_FOLD,
            upper=0.0,
        ),
    )
    model.articulation(
        "base_to_rear_right_leg",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_right_leg,
        origin=Origin(xyz=(pivot_x, -pivot_y, 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=LEG_FOLD,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_body")
    grate = object_model.get_part("cook_grate")
    lid = object_model.get_part("lid")
    front_left_leg = object_model.get_part("front_left_leg")
    front_right_leg = object_model.get_part("front_right_leg")
    rear_left_leg = object_model.get_part("rear_left_leg")
    rear_right_leg = object_model.get_part("rear_right_leg")

    lid_hinge = object_model.get_articulation("base_to_lid")
    front_left_joint = object_model.get_articulation("base_to_front_left_leg")
    rear_left_joint = object_model.get_articulation("base_to_rear_left_leg")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.003,
            name="closed lid seats on the body rim",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.26,
            name="lid covers the cook body footprint",
        )
        ctx.expect_gap(
            lid,
            grate,
            axis="z",
            min_gap=0.020,
            max_gap=0.050,
            name="closed lid clears the cooking grate",
        )
        ctx.expect_within(
            grate,
            base,
            axes="xy",
            margin=0.020,
            name="cooking grate stays inside the firebox",
        )
        ctx.expect_contact(
            grate,
            base,
            elem_a="grate_left_frame",
            elem_b="left_grate_ledge",
            name="left grate rail rests on the support ledge",
        )
        ctx.expect_contact(
            grate,
            base,
            elem_a="grate_right_frame",
            elem_b="right_grate_ledge",
            name="right grate rail rests on the support ledge",
        )

    body_aabb = ctx.part_world_aabb(base)
    for leg, label in (
        (front_left_leg, "front left"),
        (front_right_leg, "front right"),
        (rear_left_leg, "rear left"),
        (rear_right_leg, "rear right"),
    ):
        leg_aabb = ctx.part_world_aabb(leg)
        ok = (
            body_aabb is not None
            and leg_aabb is not None
            and leg_aabb[0][2] < body_aabb[0][2] - 0.085
            and leg_aabb[1][2] < body_aabb[1][2] - 0.015
        )
        ctx.check(
            f"{label} leg hangs below the cook body",
            ok,
            details=f"body_aabb={body_aabb}, leg_aabb={leg_aabb}",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    with ctx.pose({lid_hinge: math.radians(100.0)}):
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    ctx.check(
        "lid front edge swings upward on the rear hinge",
        (
            closed_front is not None
            and open_front is not None
            and open_front[1][2] > closed_front[1][2] + 0.11
        ),
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    front_left_deployed = ctx.part_world_aabb(front_left_leg)
    with ctx.pose({front_left_joint: LEG_FOLD}):
        front_left_folded = ctx.part_world_aabb(front_left_leg)
        ctx.expect_within(
            front_left_leg,
            base,
            axes="xy",
            margin=0.045,
            name="front left leg tucks beneath the base footprint",
        )
    ctx.check(
        "front left leg folds upward toward the underside",
        (
            front_left_deployed is not None
            and front_left_folded is not None
            and front_left_folded[0][2] > front_left_deployed[0][2] + 0.075
        ),
        details=f"deployed={front_left_deployed}, folded={front_left_folded}",
    )

    rear_left_deployed = ctx.part_world_aabb(rear_left_leg)
    with ctx.pose({rear_left_joint: LEG_FOLD}):
        rear_left_folded = ctx.part_world_aabb(rear_left_leg)
        ctx.expect_within(
            rear_left_leg,
            base,
            axes="xy",
            margin=0.045,
            name="rear left leg tucks beneath the base footprint",
        )
    ctx.check(
        "rear left leg folds upward toward the underside",
        (
            rear_left_deployed is not None
            and rear_left_folded is not None
            and rear_left_folded[0][2] > rear_left_deployed[0][2] + 0.075
        ),
        details=f"deployed={rear_left_deployed}, folded={rear_left_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

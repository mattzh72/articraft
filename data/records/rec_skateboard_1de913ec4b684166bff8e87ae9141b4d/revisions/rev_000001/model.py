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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


DECK_LENGTH = 0.81
DECK_WIDTH = 0.205
DECK_THICKNESS = 0.013
TRUCK_SPACING = 0.47
TRUCK_HALF_AXLE = 0.112
AXLE_DROP = 0.052
WHEEL_RADIUS = 0.027
WHEEL_WIDTH = 0.032
AXLE_RADIUS = 0.003
WHEEL_BORE_RADIUS = 0.005
WHEEL_CENTER_Y = TRUCK_HALF_AXLE - WHEEL_WIDTH * 0.5 - 0.005
WHEEL_SPACER_LENGTH = 0.002
AXLE_NUT_LENGTH = 0.005
KINGPIN_LEAN = math.radians(35.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _deck_section(
    x_pos: float,
    *,
    width: float,
    bottom_z: float,
    thickness: float,
    concave: float,
    belly: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    top_z = bottom_z + thickness
    return [
        (x_pos, -half_w, bottom_z + 0.0015),
        (x_pos, -half_w * 0.96, top_z - 0.0012),
        (x_pos, -half_w * 0.62, top_z),
        (x_pos, 0.0, top_z - concave),
        (x_pos, half_w * 0.62, top_z),
        (x_pos, half_w * 0.96, top_z - 0.0012),
        (x_pos, half_w, bottom_z + 0.0015),
        (x_pos, 0.0, bottom_z - belly),
    ]


def _build_deck_mesh():
    return section_loft(
        [
            _deck_section(
                -DECK_LENGTH * 0.5,
                width=0.090,
                bottom_z=0.084,
                thickness=0.010,
                concave=0.001,
                belly=0.0008,
            ),
            _deck_section(
                -0.33,
                width=0.170,
                bottom_z=0.056,
                thickness=0.012,
                concave=0.0015,
                belly=0.0010,
            ),
            _deck_section(
                -TRUCK_SPACING * 0.5,
                width=0.195,
                bottom_z=0.046,
                thickness=DECK_THICKNESS,
                concave=0.0024,
                belly=0.0013,
            ),
            _deck_section(
                0.0,
                width=DECK_WIDTH,
                bottom_z=0.045,
                thickness=DECK_THICKNESS,
                concave=0.0030,
                belly=0.0015,
            ),
            _deck_section(
                TRUCK_SPACING * 0.5,
                width=0.195,
                bottom_z=0.046,
                thickness=DECK_THICKNESS,
                concave=0.0024,
                belly=0.0013,
            ),
            _deck_section(
                0.33,
                width=0.170,
                bottom_z=0.057,
                thickness=0.012,
                concave=0.0015,
                belly=0.0010,
            ),
            _deck_section(
                DECK_LENGTH * 0.5,
                width=0.088,
                bottom_z=0.089,
                thickness=0.010,
                concave=0.001,
                belly=0.0008,
            ),
        ]
    )


def _add_truck_baseplate(deck_part, *, x_pos: float, truck_sign: float, deck_top, deck_bottom, steel, rubber) -> None:
    deck_part.visual(
        Box((0.082, 0.055, 0.003)),
        origin=Origin(xyz=(x_pos, 0.0, deck_bottom - 0.0015)),
        material=rubber,
        name=f"{'front' if truck_sign > 0 else 'rear'}_riser_pad",
    )
    deck_part.visual(
        Box((0.082, 0.058, 0.012)),
        origin=Origin(xyz=(x_pos, 0.0, deck_bottom - 0.0090)),
        material=steel,
        name=f"{'front' if truck_sign > 0 else 'rear'}_baseplate",
    )
    deck_part.visual(
        Cylinder(radius=0.0032, length=DECK_THICKNESS + 0.021),
        origin=Origin(xyz=(x_pos, 0.0, deck_top - 0.5 * (DECK_THICKNESS + 0.021))),
        material=steel,
        name=f"{'front' if truck_sign > 0 else 'rear'}_center_bolt",
    )


def _build_hanger_part(model: ArticulatedObject, name: str, *, truck_sign: float, painted_metal, dark_metal):
    hanger = model.part(name)
    axle_x = 0.012 * truck_sign
    inner_face_y = WHEEL_CENTER_Y - WHEEL_WIDTH * 0.5
    outer_face_y = WHEEL_CENTER_Y + WHEEL_WIDTH * 0.5
    hanger.visual(
        Cylinder(radius=0.0115, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=painted_metal,
        name="pivot_barrel",
    )
    hanger.visual(
        Box((0.036, 0.052, 0.016)),
        origin=Origin(xyz=(0.006 * truck_sign, 0.0, -0.026)),
        material=painted_metal,
        name="upper_yoke",
    )
    hanger.visual(
        Box((0.074, 0.072, 0.022)),
        origin=Origin(xyz=(0.010 * truck_sign, 0.0, -0.041)),
        material=painted_metal,
        name="hanger_body",
    )
    hanger.visual(
        Box((0.022, 0.094, 0.016)),
        origin=Origin(xyz=(axle_x, 0.0, -AXLE_DROP + 0.008)),
        material=painted_metal,
        name="axle_bridge",
    )
    hanger.visual(
        Cylinder(radius=AXLE_RADIUS, length=TRUCK_HALF_AXLE * 2.0),
        origin=Origin(
            xyz=(axle_x, 0.0, -AXLE_DROP),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_metal,
        name="axle",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        hanger.visual(
            Cylinder(radius=0.009, length=WHEEL_SPACER_LENGTH),
            origin=Origin(
                xyz=(
                    axle_x,
                    side_sign * (inner_face_y - WHEEL_SPACER_LENGTH * 0.5),
                    -AXLE_DROP,
                ),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"{side_name}_speed_ring",
        )
        hanger.visual(
            Cylinder(radius=0.0095, length=AXLE_NUT_LENGTH),
            origin=Origin(
                xyz=(
                    axle_x,
                    side_sign * (outer_face_y + AXLE_NUT_LENGTH * 0.5),
                    -AXLE_DROP,
                ),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"{side_name}_axle_nut",
        )
    hanger.inertial = Inertial.from_geometry(
        Box((0.09, 0.23, 0.08)),
        mass=0.75,
        origin=Origin(xyz=(0.004 * truck_sign, 0.0, -0.046)),
    )
    return hanger


def _build_wheel_part(model: ArticulatedObject, name: str, *, wheel_material):
    wheel = model.part(name)
    half_w = WHEEL_WIDTH * 0.5
    wheel_mesh = _save_mesh(
        f"{name}_mesh",
        LatheGeometry.from_shell_profiles(
            [
                (0.0235, -half_w),
                (0.0260, -half_w * 0.72),
                (WHEEL_RADIUS, -half_w * 0.25),
                (WHEEL_RADIUS, half_w * 0.25),
                (0.0260, half_w * 0.72),
                (0.0235, half_w),
            ],
            [
                (WHEEL_BORE_RADIUS, -half_w),
                (0.0105, -half_w * 0.55),
                (0.0115, -half_w * 0.10),
                (0.0115, half_w * 0.10),
                (0.0105, half_w * 0.55),
                (WHEEL_BORE_RADIUS, half_w),
            ],
            segments=52,
        ),
    )
    wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=wheel_material,
        name="wheel_tire",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.18,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard_standard_trucks")

    maple = model.material("maple", rgba=(0.63, 0.42, 0.24, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.08, 1.0))
    truck_silver = model.material("truck_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    axle_dark = model.material("axle_dark", rgba=(0.24, 0.24, 0.26, 1.0))
    riser_rubber = model.material("riser_rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.93, 0.95, 0.90, 1.0))

    deck = model.part("deck")
    deck_mesh = _save_mesh("skateboard_deck", _build_deck_mesh())
    deck.visual(deck_mesh, material=maple, name="deck_shell")
    deck.visual(
        Box((0.58, DECK_WIDTH * 0.88, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0585)),
        material=grip_black,
        name="griptape_center",
    )
    _add_truck_baseplate(
        deck,
        x_pos=TRUCK_SPACING * 0.5,
        truck_sign=1.0,
        deck_top=0.058,
        deck_bottom=0.046,
        steel=truck_silver,
        rubber=riser_rubber,
    )
    _add_truck_baseplate(
        deck,
        x_pos=-TRUCK_SPACING * 0.5,
        truck_sign=-1.0,
        deck_top=0.058,
        deck_bottom=0.046,
        steel=truck_silver,
        rubber=riser_rubber,
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.070)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    front_hanger = _build_hanger_part(
        model,
        "front_hanger",
        truck_sign=1.0,
        painted_metal=truck_silver,
        dark_metal=axle_dark,
    )
    rear_hanger = _build_hanger_part(
        model,
        "rear_hanger",
        truck_sign=-1.0,
        painted_metal=truck_silver,
        dark_metal=axle_dark,
    )

    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_hanger,
        origin=Origin(xyz=(TRUCK_SPACING * 0.5, 0.0, 0.034)),
        axis=(-math.sin(KINGPIN_LEAN), 0.0, math.cos(KINGPIN_LEAN)),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=4.0,
            lower=math.radians(-25.0),
            upper=math.radians(25.0),
        ),
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_hanger,
        origin=Origin(xyz=(-TRUCK_SPACING * 0.5, 0.0, 0.034)),
        axis=(math.sin(KINGPIN_LEAN), 0.0, math.cos(KINGPIN_LEAN)),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=4.0,
            lower=math.radians(-25.0),
            upper=math.radians(25.0),
        ),
    )

    for truck_name, hanger, truck_sign in (
        ("front", front_hanger, 1.0),
        ("rear", rear_hanger, -1.0),
    ):
        for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
            wheel = _build_wheel_part(
                model,
                f"{truck_name}_{side_name}_wheel",
                wheel_material=wheel_urethane,
            )
            model.articulation(
                f"{truck_name}_{side_name}_wheel_spin",
                ArticulationType.CONTINUOUS,
                parent=hanger,
                child=wheel,
                origin=Origin(
                    xyz=(0.012 * truck_sign, side_sign * WHEEL_CENTER_Y, -AXLE_DROP),
                    rpy=(0.0, 0.0, 0.0),
                ),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=6.0,
                    velocity=45.0,
                ),
            )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_hanger = object_model.get_part("front_hanger")
    rear_hanger = object_model.get_part("rear_hanger")
    front_left = object_model.get_part("front_left_wheel")
    front_right = object_model.get_part("front_right_wheel")
    rear_left = object_model.get_part("rear_left_wheel")
    rear_right = object_model.get_part("rear_right_wheel")
    front_steer = object_model.get_articulation("front_truck_steer")
    rear_steer = object_model.get_articulation("rear_truck_steer")

    for truck_name, hanger in (("front", front_hanger), ("rear", rear_hanger)):
        for side_name in ("left", "right"):
            wheel = object_model.get_part(f"{truck_name}_{side_name}_wheel")
            spin = object_model.get_articulation(f"{truck_name}_{side_name}_wheel_spin")
            ctx.expect_contact(
                wheel,
                hanger,
                elem_a="wheel_tire",
                elem_b=f"{side_name}_speed_ring",
                name=f"{truck_name} {side_name} wheel seats on inboard speed ring",
            )
            ctx.expect_contact(
                wheel,
                hanger,
                elem_a="wheel_tire",
                elem_b=f"{side_name}_axle_nut",
                name=f"{truck_name} {side_name} wheel is retained by axle nut",
            )
            ctx.check(
                f"{truck_name} {side_name} wheel spin axis follows axle",
                spin.joint_type == ArticulationType.CONTINUOUS
                and spin.axis == (0.0, 1.0, 0.0)
                and spin.motion_limits is not None
                and spin.motion_limits.lower is None
                and spin.motion_limits.upper is None,
                details=f"type={spin.joint_type}, axis={spin.axis}, limits={spin.motion_limits}",
            )

    def expect_pair_mirrored(left_part, right_part, label: str) -> None:
        left_pos = ctx.part_world_position(left_part)
        right_pos = ctx.part_world_position(right_part)
        ok = (
            left_pos is not None
            and right_pos is not None
            and abs(left_pos[0] - right_pos[0]) < 0.002
            and abs(left_pos[1] + right_pos[1]) < 0.002
            and abs(left_pos[2] - right_pos[2]) < 0.002
        )
        ctx.check(label, ok, details=f"left={left_pos}, right={right_pos}")

    expect_pair_mirrored(front_left, front_right, "front wheels are mirrored about the deck centerline")
    expect_pair_mirrored(rear_left, rear_right, "rear wheels are mirrored about the deck centerline")

    front_axis = front_steer.axis
    rear_axis = rear_steer.axis
    ctx.check(
        "front and rear kingpin axes are mirrored",
        abs(front_axis[0] + rear_axis[0]) < 1e-9
        and abs(front_axis[1] - rear_axis[1]) < 1e-9
        and abs(front_axis[2] - rear_axis[2]) < 1e-9,
        details=f"front={front_axis}, rear={rear_axis}",
    )

    with ctx.pose({front_steer: 0.30, rear_steer: 0.30}):
        front_left_pos = ctx.part_world_position(front_left)
        front_right_pos = ctx.part_world_position(front_right)
        rear_left_pos = ctx.part_world_position(rear_left)
        rear_right_pos = ctx.part_world_position(rear_right)
        ctx.check(
            "positive steer yaws the front truck across the axle line",
            front_left_pos is not None
            and front_right_pos is not None
            and front_left_pos[0] < front_right_pos[0] - 0.02,
            details=f"front_left={front_left_pos}, front_right={front_right_pos}",
        )
        ctx.check(
            "positive steer yaws the rear truck across the axle line",
            rear_left_pos is not None
            and rear_right_pos is not None
            and rear_left_pos[0] < rear_right_pos[0] - 0.02,
            details=f"rear_left={rear_left_pos}, rear_right={rear_right_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

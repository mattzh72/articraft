from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_tongue_plate():
    outer_profile = [
        (-0.040, -0.012),
        (-0.006, -0.012),
        (0.016, -0.0105),
        (0.024, -0.0075),
        (0.0285, -0.0030),
        (0.0295, 0.0000),
        (0.0285, 0.0030),
        (0.0240, 0.0075),
        (0.0160, 0.0105),
        (-0.006, 0.012),
        (-0.040, 0.012),
    ]
    belt_hole = _translated_profile(
        rounded_rect_profile(0.018, 0.010, 0.0028, corner_segments=6),
        dx=-0.023,
    )
    latch_hole = _translated_profile(
        rounded_rect_profile(0.008, 0.0048, 0.0010, corner_segments=5),
        dx=0.024,
    )
    tongue_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [belt_hole, latch_hole],
        0.0024,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(tongue_geom, "tongue_plate")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="seat_belt_buckle_clicker")

    housing_black = model.material("housing_black", rgba=(0.14, 0.14, 0.15, 1.0))
    housing_charcoal = model.material("housing_charcoal", rgba=(0.21, 0.22, 0.24, 1.0))
    button_red = model.material("button_red", rgba=(0.79, 0.10, 0.08, 1.0))
    button_dark = model.material("button_dark", rgba=(0.52, 0.06, 0.05, 1.0))
    tongue_metal = model.material("tongue_metal", rgba=(0.82, 0.84, 0.86, 1.0))

    housing_length = 0.055
    housing_width = 0.034
    housing_height = 0.017
    wall_thickness = 0.003
    floor_thickness = 0.003
    roof_thickness = 0.003

    housing = model.part("housing")
    housing.visual(
        Box((housing_length, housing_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=housing_black,
        name="floor",
    )
    housing.visual(
        Box((housing_length, wall_thickness, housing_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_width * 0.5 - wall_thickness * 0.5,
                housing_height * 0.5,
            )
        ),
        material=housing_charcoal,
        name="side_wall_0",
    )
    housing.visual(
        Box((housing_length, wall_thickness, housing_height)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_width * 0.5 + wall_thickness * 0.5,
                housing_height * 0.5,
            )
        ),
        material=housing_charcoal,
        name="side_wall_1",
    )
    housing.visual(
        Box((wall_thickness, housing_width - 2.0 * wall_thickness, housing_height)),
        origin=Origin(
            xyz=(
                housing_length * 0.5 - wall_thickness * 0.5,
                0.0,
                housing_height * 0.5,
            )
        ),
        material=housing_charcoal,
        name="rear_wall",
    )
    housing.visual(
        Box((0.014, housing_width - 2.0 * wall_thickness, roof_thickness)),
        origin=Origin(xyz=(-0.0205, 0.0, housing_height - roof_thickness * 0.5)),
        material=housing_charcoal,
        name="front_bridge",
    )
    housing.visual(
        Box((0.018, housing_width - 2.0 * wall_thickness, roof_thickness)),
        origin=Origin(xyz=(0.0185, 0.0, housing_height - roof_thickness * 0.5)),
        material=housing_charcoal,
        name="rear_bridge",
    )
    housing.visual(
        Box((0.023, 0.008, roof_thickness)),
        origin=Origin(
            xyz=(0.0010, 0.0100 + 0.0040, housing_height - roof_thickness * 0.5)
        ),
        material=housing_charcoal,
        name="button_bezel_0",
    )
    housing.visual(
        Box((0.023, 0.008, roof_thickness)),
        origin=Origin(
            xyz=(0.0010, -0.0100 - 0.0040, housing_height - roof_thickness * 0.5)
        ),
        material=housing_charcoal,
        name="button_bezel_1",
    )
    housing.visual(
        Box((0.010, housing_width - 2.0 * wall_thickness, 0.0035)),
        origin=Origin(xyz=(-0.0225, 0.0, 0.01225)),
        material=housing_black,
        name="slot_lip",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_length, housing_width, housing_height)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    tongue = model.part("tongue")
    tongue.visual(_build_tongue_plate(), material=tongue_metal, name="plate")
    tongue.inertial = Inertial.from_geometry(
        Box((0.0695, 0.024, 0.0024)),
        mass=0.018,
        origin=Origin(xyz=(-0.00525, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_tongue",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tongue,
        origin=Origin(xyz=(-housing_length * 0.5, 0.0, 0.0042)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.10,
            lower=0.0,
            upper=0.018,
        ),
    )

    button = model.part("button")
    button.visual(
        Box((0.019, 0.015, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=button_red,
        name="cap",
    )
    button.visual(
        Box((0.023, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=button_dark,
        name="stem",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.023, 0.020, 0.012)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    model.articulation(
        "housing_to_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button,
        origin=Origin(xyz=(-0.002, 0.0, housing_height)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    tongue = object_model.get_part("tongue")
    button = object_model.get_part("button")
    tongue_slide = object_model.get_articulation("housing_to_tongue")
    button_slide = object_model.get_articulation("housing_to_button")

    ctx.expect_within(
        tongue,
        housing,
        axes="yz",
        margin=0.0,
        name="tongue stays centered in slot at rest",
    )
    ctx.expect_overlap(
        tongue,
        housing,
        axes="x",
        min_overlap=0.028,
        name="tongue remains substantially inserted at rest",
    )
    ctx.expect_within(
        button,
        housing,
        axes="xy",
        margin=0.0,
        name="button stays within housing footprint at rest",
    )

    rest_tongue_pos = ctx.part_world_position(tongue)
    rest_button_pos = ctx.part_world_position(button)

    with ctx.pose({tongue_slide: 0.018}):
        ctx.expect_within(
            tongue,
            housing,
            axes="yz",
            margin=0.0,
            name="tongue stays centered in slot when pulled",
        )
        ctx.expect_overlap(
            tongue,
            housing,
            axes="x",
            min_overlap=0.010,
            name="tongue retains insertion at max pull",
        )
        pulled_tongue_pos = ctx.part_world_position(tongue)

    with ctx.pose({button_slide: 0.003}):
        ctx.expect_within(
            button,
            housing,
            axes="xy",
            margin=0.0,
            name="button stays guided while pressed",
        )
        pressed_button_pos = ctx.part_world_position(button)

    ctx.check(
        "tongue positive travel pulls outward",
        rest_tongue_pos is not None
        and pulled_tongue_pos is not None
        and pulled_tongue_pos[0] < rest_tongue_pos[0] - 0.015,
        details=f"rest={rest_tongue_pos}, pulled={pulled_tongue_pos}",
    )
    ctx.check(
        "button positive travel depresses downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.0025,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


HOUSING_WIDTH = 0.340
HOUSING_DEPTH = 0.240
HOUSING_HEIGHT = 0.060
WALL_THICKNESS = 0.004

INNER_WIDTH = HOUSING_WIDTH - 2.0 * WALL_THICKNESS
INNER_DEPTH = HOUSING_DEPTH - 2.0 * WALL_THICKNESS
FRONT_FACE_Y = -HOUSING_DEPTH * 0.5 + WALL_THICKNESS * 0.5
REAR_FACE_Y = HOUSING_DEPTH * 0.5 - WALL_THICKNESS * 0.5
SIDE_FACE_X = HOUSING_WIDTH * 0.5 - WALL_THICKNESS * 0.5
SIDE_WALL_HEIGHT = HOUSING_HEIGHT - 2.0 * WALL_THICKNESS
SIDE_WALL_CENTER_Z = WALL_THICKNESS + SIDE_WALL_HEIGHT * 0.5
TOP_DECK_CENTER_Z = HOUSING_HEIGHT - WALL_THICKNESS * 0.5

CHANNEL_XS = (-0.105, -0.035, 0.035, 0.105)
SLOT_WIDTH = 0.010
SLOT_Z_MIN = 0.011
SLOT_Z_MAX = 0.051
FADER_GUIDE_HEIGHT = 0.018
FADER_TRAVEL_LOWER = -0.012
FADER_TRAVEL_UPPER = 0.010
FADER_JOINT_Z = 0.032
KNOB_PANEL_Y = 0.030


def _front_band_spans() -> list[tuple[float, float]]:
    left_inner = -HOUSING_WIDTH * 0.5 + WALL_THICKNESS
    right_inner = HOUSING_WIDTH * 0.5 - WALL_THICKNESS
    slot_lefts = [center_x - SLOT_WIDTH * 0.5 for center_x in CHANNEL_XS]
    slot_rights = [center_x + SLOT_WIDTH * 0.5 for center_x in CHANNEL_XS]

    spans: list[tuple[float, float]] = [(left_inner, slot_lefts[0])]
    for index in range(len(CHANNEL_XS) - 1):
        spans.append((slot_rights[index], slot_lefts[index + 1]))
    spans.append((slot_rights[-1], right_inner))
    return spans


def _add_housing(housing, housing_mat, panel_mat, rubber_mat) -> None:
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, WALL_THICKNESS * 0.5)),
        material=panel_mat,
        name="bottom_plate",
    )
    housing.visual(
        Box((INNER_WIDTH, INNER_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_DECK_CENTER_Z)),
        material=housing_mat,
        name="top_deck",
    )
    housing.visual(
        Box((WALL_THICKNESS, INNER_DEPTH, SIDE_WALL_HEIGHT)),
        origin=Origin(xyz=(-SIDE_FACE_X, 0.0, SIDE_WALL_CENTER_Z)),
        material=housing_mat,
        name="left_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, INNER_DEPTH, SIDE_WALL_HEIGHT)),
        origin=Origin(xyz=(SIDE_FACE_X, 0.0, SIDE_WALL_CENTER_Z)),
        material=housing_mat,
        name="right_wall",
    )
    housing.visual(
        Box((INNER_WIDTH, WALL_THICKNESS, SIDE_WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, REAR_FACE_Y, SIDE_WALL_CENTER_Z)),
        material=housing_mat,
        name="rear_wall",
    )

    lower_rail_height = SLOT_Z_MIN - WALL_THICKNESS
    upper_rail_height = HOUSING_HEIGHT - WALL_THICKNESS - SLOT_Z_MAX
    housing.visual(
        Box((INNER_WIDTH, WALL_THICKNESS, lower_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_FACE_Y,
                WALL_THICKNESS + lower_rail_height * 0.5,
            )
        ),
        material=housing_mat,
        name="front_lower_rail",
    )
    housing.visual(
        Box((INNER_WIDTH, WALL_THICKNESS, upper_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_FACE_Y,
                SLOT_Z_MAX + upper_rail_height * 0.5,
            )
        ),
        material=housing_mat,
        name="front_upper_rail",
    )

    slot_band_height = SLOT_Z_MAX - SLOT_Z_MIN
    slot_band_center_z = (SLOT_Z_MIN + SLOT_Z_MAX) * 0.5
    for index, (x_min, x_max) in enumerate(_front_band_spans()):
        housing.visual(
            Box((x_max - x_min, WALL_THICKNESS, slot_band_height)),
            origin=Origin(
                xyz=(
                    (x_min + x_max) * 0.5,
                    FRONT_FACE_Y,
                    slot_band_center_z,
                )
            ),
            material=housing_mat,
            name=f"front_band_{index}",
        )

    track_depth = 0.014
    track_width = 0.0025
    track_center_y = FRONT_FACE_Y + WALL_THICKNESS * 0.5 + track_depth * 0.5
    for index, slot_x in enumerate(CHANNEL_XS, start=1):
        for side_name, x_offset in (("left", -0.00625), ("right", 0.00625)):
            housing.visual(
                Box((track_width, track_depth, slot_band_height)),
                origin=Origin(
                    xyz=(
                        slot_x + x_offset,
                        track_center_y,
                        slot_band_center_z,
                    )
                ),
                material=panel_mat,
                name=f"slider_track_{index}_{side_name}",
            )

    housing.visual(
        Box((0.288, 0.150, 0.0015)),
        origin=Origin(xyz=(0.0, 0.016, HOUSING_HEIGHT + 0.00075)),
        material=panel_mat,
        name="top_panel_inset",
    )
    housing.visual(
        Box((0.250, 0.040, 0.0015)),
        origin=Origin(xyz=(0.0, -0.070, HOUSING_HEIGHT + 0.00075)),
        material=panel_mat,
        name="front_panel_inset",
    )

    for foot_x in (-0.120, 0.120):
        for foot_y in (-0.080, 0.080):
            housing.visual(
                Box((0.030, 0.022, 0.003)),
                origin=Origin(xyz=(foot_x, foot_y, 0.0015)),
                material=rubber_mat,
                name=f"foot_{'l' if foot_x < 0.0 else 'r'}_{'f' if foot_y < 0.0 else 'b'}",
            )


def _add_isolator_knob(model, parent, knob_name: str, joint_name: str, x_pos: float) -> None:
    knob = model.part(knob_name)
    knob.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="knob_black",
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material="knob_black",
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material="knob_center",
        name="knob_center",
    )
    knob.visual(
        Box((0.003, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.006, 0.034)),
        material="indicator_white",
        name="indicator_line",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.036),
        mass=0.050,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent=parent,
        child=knob,
        origin=Origin(xyz=(x_pos, KNOB_PANEL_Y, HOUSING_HEIGHT + 0.0015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=10.0),
    )


def _add_channel_fader(model, parent, fader_name: str, joint_name: str, x_pos: float) -> None:
    fader = model.part(fader_name)
    fader.visual(
        Box((0.006, 0.014, FADER_GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        material="slider_dark",
        name="fader_guide",
    )
    fader.visual(
        Box((0.010, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material="slider_dark",
        name="fader_shoe",
    )
    fader.visual(
        Box((0.020, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material="slider_silver",
        name="fader_cap",
    )
    fader.visual(
        Box((0.012, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material="slider_light",
        name="fader_grip",
    )
    fader.inertial = Inertial.from_geometry(
        Box((0.020, 0.032, 0.030)),
        mass=0.035,
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=parent,
        child=fader,
        origin=Origin(xyz=(x_pos, FRONT_FACE_Y, FADER_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.15,
            lower=FADER_TRAVEL_LOWER,
            upper=FADER_TRAVEL_UPPER,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_rotary_isolator_mixer")

    model.material("housing_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("knob_center", rgba=(0.24, 0.24, 0.26, 1.0))
    model.material("indicator_white", rgba=(0.92, 0.92, 0.90, 1.0))
    model.material("slider_silver", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("slider_light", rgba=(0.90, 0.90, 0.90, 1.0))
    model.material("slider_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    housing = model.part("housing")
    _add_housing(
        housing,
        housing_mat="housing_charcoal",
        panel_mat="panel_black",
        rubber_mat="rubber_black",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT * 0.5)),
    )

    for index, x_pos in enumerate(CHANNEL_XS, start=1):
        _add_channel_fader(
            model,
            housing,
            fader_name=f"channel_fader_{index}",
            joint_name=f"housing_to_channel_fader_{index}",
            x_pos=x_pos,
        )
        _add_isolator_knob(
            model,
            housing,
            knob_name=f"isolator_knob_{index}",
            joint_name=f"housing_to_isolator_knob_{index}",
            x_pos=x_pos,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    front_lower_rail = housing.get_visual("front_lower_rail")
    front_upper_rail = housing.get_visual("front_upper_rail")
    top_deck = housing.get_visual("top_deck")

    def _aabb_z_bounds(part_obj, elem_name: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][2], aabb[1][2])

    lower_bounds = _aabb_z_bounds(housing, front_lower_rail.name)
    upper_bounds = _aabb_z_bounds(housing, front_upper_rail.name)

    for index in range(1, 5):
        knob = object_model.get_part(f"isolator_knob_{index}")
        knob_joint = object_model.get_articulation(f"housing_to_isolator_knob_{index}")
        knob_limits = knob_joint.motion_limits

        ctx.check(
            f"isolator knob {index} uses continuous vertical rotation",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS
            and knob_joint.axis == (0.0, 0.0, 1.0)
            and knob_limits is not None
            and knob_limits.lower is None
            and knob_limits.upper is None,
            details=(
                f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, "
                f"limits={knob_limits}"
            ),
        )
        ctx.expect_gap(
            knob,
            housing,
            axis="z",
            positive_elem="knob_skirt",
            negative_elem=top_deck.name,
            max_gap=0.002,
            max_penetration=0.0,
            name=f"isolator knob {index} seats on the top deck",
        )

        fader = object_model.get_part(f"channel_fader_{index}")
        fader_joint = object_model.get_articulation(f"housing_to_channel_fader_{index}")
        fader_limits = fader_joint.motion_limits

        ctx.check(
            f"channel fader {index} uses vertical prismatic travel",
            fader_joint.articulation_type == ArticulationType.PRISMATIC
            and fader_joint.axis == (0.0, 0.0, 1.0)
            and fader_limits is not None
            and fader_limits.lower is not None
            and fader_limits.upper is not None,
            details=(
                f"type={fader_joint.articulation_type}, axis={fader_joint.axis}, "
                f"limits={fader_limits}"
            ),
        )

        rest_position = ctx.part_world_position(fader)
        with ctx.pose({fader_joint: fader_limits.upper}):
            high_position = ctx.part_world_position(fader)
            guide_bounds_high = _aabb_z_bounds(fader, "fader_guide")
        with ctx.pose({fader_joint: fader_limits.lower}):
            low_position = ctx.part_world_position(fader)
            guide_bounds_low = _aabb_z_bounds(fader, "fader_guide")

        ctx.check(
            f"channel fader {index} moves upward through its slot",
            rest_position is not None
            and high_position is not None
            and low_position is not None
            and high_position[2] > low_position[2] + 0.015,
            details=f"low={low_position}, rest={rest_position}, high={high_position}",
        )

        ctx.check(
            f"channel fader {index} guide stays between the slot rails",
            lower_bounds is not None
            and upper_bounds is not None
            and guide_bounds_low is not None
            and guide_bounds_high is not None
            and guide_bounds_low[0] >= lower_bounds[1] - 1e-6
            and guide_bounds_high[1] <= upper_bounds[0] + 1e-6,
            details=(
                f"lower_rail={lower_bounds}, upper_rail={upper_bounds}, "
                f"guide_low={guide_bounds_low}, guide_high={guide_bounds_high}"
            ),
        )

        ctx.expect_gap(
            housing,
            fader,
            axis="y",
            positive_elem="front_lower_rail",
            negative_elem="fader_cap",
            max_gap=0.012,
            max_penetration=0.0,
            name=f"channel fader {index} cap stays proud of the front face",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    TorusGeometry,
    mesh_from_geometry,
    place_on_face,
)


HOUSING_SIZE = (0.34, 0.22, 0.045)
DECK_SIZE = (0.318, 0.198, 0.010)
TOP_Z = 0.055


def _build_knob(model: ArticulatedObject, housing, *, name: str, x_pos: float, y_pos: float):
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="soft_black",
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="knob_black",
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="knob_cap",
        name="knob_cap",
    )
    knob.visual(
        Box((0.0025, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, 0.007, 0.0218)),
        material="indicator_white",
        name="knob_pointer",
    )

    joint = model.articulation(
        f"housing_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=place_on_face(housing, "+z", face_pos=(x_pos, y_pos), proud=0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    return knob, joint


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_effects_unit")

    model.material("chassis_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("deck_black", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("panel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("soft_black", rgba=(0.09, 0.09, 0.09, 1.0))
    model.material("knob_black", rgba=(0.14, 0.14, 0.14, 1.0))
    model.material("knob_cap", rgba=(0.20, 0.20, 0.20, 1.0))
    model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("platter_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("metal_gray", rgba=(0.36, 0.37, 0.39, 1.0))
    model.material("indicator_white", rgba=(0.92, 0.92, 0.92, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box(HOUSING_SIZE),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_SIZE[2] / 2.0)),
        material="chassis_black",
        name="body_shell",
    )
    housing.visual(
        Box(DECK_SIZE),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material="deck_black",
        name="top_deck",
    )
    housing.visual(
        Box((0.318, 0.060, 0.002)),
        origin=Origin(xyz=(0.0, -0.072, 0.054)),
        material="panel_black",
        name="front_panel_strip",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.055)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    jog_wheel = model.part("jog_wheel")
    jog_rim = TorusGeometry(
        radius=0.082,
        tube=0.010,
        radial_segments=24,
        tubular_segments=40,
    )
    jog_rim.translate(0.0, 0.0, 0.014)
    jog_wheel.visual(
        Cylinder(radius=0.088, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="rubber_black",
        name="bearing_ring",
    )
    jog_wheel.visual(
        mesh_from_geometry(jog_rim, "jog_wheel_rim"),
        material="rubber_black",
        name="rim",
    )
    jog_wheel.visual(
        Cylinder(radius=0.085, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material="metal_gray",
        name="wheel_body",
    )
    jog_wheel.visual(
        Cylinder(radius=0.070, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material="platter_gray",
        name="platter_top",
    )
    jog_wheel.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="metal_gray",
        name="center_hub",
    )
    jog_wheel.visual(
        Box((0.004, 0.014, 0.0012)),
        origin=Origin(xyz=(0.0, 0.059, 0.022)),
        material="indicator_white",
        name="index_marker",
    )

    model.articulation(
        "housing_to_jog_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=jog_wheel,
        origin=place_on_face(housing, "+z", face_pos=(0.0, 0.035), proud=0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=14.0),
    )

    _build_knob(model, housing, name="knob_1", x_pos=-0.105, y_pos=-0.074)
    _build_knob(model, housing, name="knob_2", x_pos=-0.035, y_pos=-0.074)
    _build_knob(model, housing, name="knob_3", x_pos=0.035, y_pos=-0.074)
    _build_knob(model, housing, name="knob_4", x_pos=0.105, y_pos=-0.074)

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
    jog_wheel = object_model.get_part("jog_wheel")
    knobs = [object_model.get_part(f"knob_{idx}") for idx in range(1, 5)]

    jog_joint = object_model.get_articulation("housing_to_jog_wheel")
    knob_joints = [
        object_model.get_articulation(f"housing_to_knob_{idx}") for idx in range(1, 5)
    ]

    all_joints = [jog_joint, *knob_joints]
    for joint in all_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a continuous vertical spin",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={joint.articulation_type}, axis={joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )

    ctx.expect_gap(
        jog_wheel,
        housing,
        axis="z",
        min_gap=0.0,
        max_gap=0.0002,
        name="jog wheel bearing ring stays seated on the deck",
    )
    ctx.expect_overlap(
        jog_wheel,
        housing,
        axes="xy",
        min_overlap=0.16,
        name="jog wheel sits broadly over the housing deck",
    )

    for idx, knob in enumerate(knobs, start=1):
        ctx.expect_gap(
            knob,
            housing,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            name=f"knob {idx} skirt stays seated on the front panel",
        )
        ctx.expect_overlap(
            knob,
            housing,
            axes="xy",
            min_overlap=0.03,
            name=f"knob {idx} is mounted over the panel footprint",
        )

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    jog_position = ctx.part_world_position(jog_wheel)
    knobs_in_row = (
        jog_position is not None
        and all(pos is not None for pos in knob_positions)
        and all(abs(pos[1] - knob_positions[0][1]) <= 0.001 for pos in knob_positions[1:])
        and all(
            knob_positions[i][0] < knob_positions[i + 1][0] - 0.03
            for i in range(len(knob_positions) - 1)
        )
        and all(pos[1] < jog_position[1] - 0.06 for pos in knob_positions)
    )
    ctx.check(
        "four knobs form a front control row ahead of the jog wheel",
        knobs_in_row,
        details=f"knob_positions={knob_positions}, jog_position={jog_position}",
    )

    with ctx.pose(
        {
            jog_joint: 1.2,
            knob_joints[0]: 0.7,
            knob_joints[1]: -0.9,
            knob_joints[2]: 1.6,
            knob_joints[3]: 0.35,
        }
    ):
        ctx.expect_gap(
            jog_wheel,
            housing,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            name="jog wheel remains deck-mounted while spinning",
        )
        for idx, knob in enumerate(knobs, start=1):
            ctx.expect_gap(
                knob,
                housing,
                axis="z",
                min_gap=0.0,
                max_gap=0.0002,
                name=f"knob {idx} remains upright while spinning",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

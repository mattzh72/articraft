from __future__ import annotations

from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PANEL_Z = 0.058
SLOT_TOP_Z = 0.059


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


CASE_MAT = _mat("satin_black_case", (0.015, 0.016, 0.018, 1.0))
PANEL_MAT = _mat("dark_anodized_panel", (0.085, 0.088, 0.095, 1.0))
SLOT_MAT = _mat("black_recessed_slots", (0.002, 0.002, 0.003, 1.0))
GREY_CAP_MAT = _mat("warm_grey_fader_caps", (0.62, 0.60, 0.55, 1.0))
MASTER_CAP_MAT = _mat("red_master_fader_cap", (0.75, 0.055, 0.04, 1.0))
KNOB_MAT = _mat("matte_black_knobs", (0.018, 0.018, 0.020, 1.0))
WHITE_MAT = _mat("white_control_marks", (0.92, 0.91, 0.84, 1.0))
BLUE_MAT = _mat("blue_eq_row_marks", (0.06, 0.24, 0.80, 1.0))
SCREW_MAT = _mat("brushed_screw_heads", (0.48, 0.48, 0.44, 1.0))


CHANNEL_FADER_XS = (-0.150, -0.050, 0.050, 0.150)
MASTER_FADER_X = 0.198
CHANNEL_SLOT_Y = 0.000
MASTER_SLOT_Y = 0.012
CROSSFADER_Y = -0.118
EQ_KNOB_POSITIONS = (
    (-0.085, 0.116),
    (0.085, 0.116),
    (-0.085, 0.080),
    (0.085, 0.080),
    (-0.085, 0.044),
    (0.085, 0.044),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="dj_mixer",
        materials=[
            CASE_MAT,
            PANEL_MAT,
            SLOT_MAT,
            GREY_CAP_MAT,
            MASTER_CAP_MAT,
            KNOB_MAT,
            WHITE_MAT,
            BLUE_MAT,
            SCREW_MAT,
        ],
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.460, 0.320, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=CASE_MAT,
        name="rectangular_housing",
    )
    housing.visual(
        Box((0.440, 0.300, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=PANEL_MAT,
        name="front_panel_face",
    )

    # Raised protective lip around the top control surface.
    housing.visual(
        Box((0.448, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.154, 0.060)),
        material=CASE_MAT,
        name="rear_lip",
    )
    housing.visual(
        Box((0.448, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.154, 0.060)),
        material=CASE_MAT,
        name="front_lip",
    )
    housing.visual(
        Box((0.008, 0.300, 0.004)),
        origin=Origin(xyz=(-0.224, 0.0, 0.060)),
        material=CASE_MAT,
        name="side_lip_0",
    )
    housing.visual(
        Box((0.008, 0.300, 0.004)),
        origin=Origin(xyz=(0.224, 0.0, 0.060)),
        material=CASE_MAT,
        name="side_lip_1",
    )

    # Five long recessed fader tracks plus the center crossfader slot.
    for index, x in enumerate(CHANNEL_FADER_XS):
        housing.visual(
            Box((0.014, 0.132, 0.001)),
            origin=Origin(xyz=(x, CHANNEL_SLOT_Y, PANEL_Z + 0.0005)),
            material=SLOT_MAT,
            name=f"channel_slot_{index}",
        )
        # White end marks around each travel slot.
        housing.visual(
            Box((0.036, 0.003, 0.0006)),
            origin=Origin(xyz=(x, CHANNEL_SLOT_Y + 0.073, PANEL_Z + 0.0003)),
            material=WHITE_MAT,
            name=f"channel_top_mark_{index}",
        )
        housing.visual(
            Box((0.036, 0.003, 0.0006)),
            origin=Origin(xyz=(x, CHANNEL_SLOT_Y - 0.073, PANEL_Z + 0.0003)),
            material=WHITE_MAT,
            name=f"channel_bottom_mark_{index}",
        )

    housing.visual(
        Box((0.014, 0.132, 0.001)),
        origin=Origin(xyz=(MASTER_FADER_X, MASTER_SLOT_Y, PANEL_Z + 0.0005)),
        material=SLOT_MAT,
        name="master_slot",
    )
    housing.visual(
        Box((0.036, 0.003, 0.0006)),
        origin=Origin(xyz=(MASTER_FADER_X, MASTER_SLOT_Y + 0.073, PANEL_Z + 0.0003)),
        material=WHITE_MAT,
        name="master_top_mark",
    )
    housing.visual(
        Box((0.036, 0.003, 0.0006)),
        origin=Origin(xyz=(MASTER_FADER_X, MASTER_SLOT_Y - 0.073, PANEL_Z + 0.0003)),
        material=WHITE_MAT,
        name="master_bottom_mark",
    )

    housing.visual(
        Box((0.184, 0.014, 0.001)),
        origin=Origin(xyz=(0.0, CROSSFADER_Y, PANEL_Z + 0.0005)),
        material=SLOT_MAT,
        name="crossfader_slot",
    )
    housing.visual(
        Box((0.003, 0.040, 0.0006)),
        origin=Origin(xyz=(-0.100, CROSSFADER_Y, PANEL_Z + 0.0003)),
        material=WHITE_MAT,
        name="crossfader_left_mark",
    )
    housing.visual(
        Box((0.003, 0.040, 0.0006)),
        origin=Origin(xyz=(0.100, CROSSFADER_Y, PANEL_Z + 0.0003)),
        material=WHITE_MAT,
        name="crossfader_right_mark",
    )

    # EQ section guide marks: two knob columns and three blue row cues.
    for row, y in enumerate((0.116, 0.080, 0.044)):
        housing.visual(
            Box((0.008, 0.0025, 0.0006)),
            origin=Origin(xyz=(0.0, y, PANEL_Z + 0.0003)),
            material=BLUE_MAT,
            name=f"eq_row_mark_{row}",
        )

    for index, (x, y) in enumerate(((-0.205, 0.132), (0.205, 0.132), (-0.205, -0.132), (0.205, -0.132))):
        housing.visual(
            Box((0.016, 0.016, 0.002)),
            origin=Origin(xyz=(x, y, PANEL_Z + 0.001)),
            material=SCREW_MAT,
            name=f"corner_screw_{index}",
        )

    def add_vertical_fader(
        name: str,
        x: float,
        y: float,
        slot_name: str,
        material: Material,
        travel: float,
    ) -> None:
        fader = model.part(name)
        fader.visual(
            Box((0.027, 0.020, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=SLOT_MAT,
            name="slider_shoe",
        )
        fader.visual(
            Box((0.040, 0.024, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
            material=material,
            name="fader_cap",
        )
        fader.visual(
            Box((0.0025, 0.018, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0185)),
            material=WHITE_MAT,
            name="cap_center_groove",
        )
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, y, SLOT_TOP_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.30, lower=-travel, upper=travel),
            meta={"slot": slot_name},
        )

    def add_crossfader() -> None:
        fader = model.part("crossfader")
        fader.visual(
            Box((0.026, 0.028, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=SLOT_MAT,
            name="slider_shoe",
        )
        fader.visual(
            Box((0.030, 0.046, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
            material=GREY_CAP_MAT,
            name="fader_cap",
        )
        fader.visual(
            Box((0.018, 0.0025, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0185)),
            material=WHITE_MAT,
            name="cap_center_groove",
        )
        model.articulation(
            "housing_to_crossfader",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(0.0, CROSSFADER_Y, SLOT_TOP_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.40, lower=-0.065, upper=0.065),
            meta={"slot": "crossfader_slot"},
        )

    add_crossfader()
    for index, x in enumerate(CHANNEL_FADER_XS):
        add_vertical_fader(
            f"channel_fader_{index}",
            x,
            CHANNEL_SLOT_Y,
            f"channel_slot_{index}",
            GREY_CAP_MAT,
            0.052,
        )
    add_vertical_fader(
        "master_fader",
        MASTER_FADER_X,
        MASTER_SLOT_Y,
        "master_slot",
        MASTER_CAP_MAT,
        0.052,
    )

    knob_meshes = [
        mesh_from_geometry(
            KnobGeometry(
                0.028,
                0.018,
                body_style="faceted",
                base_diameter=0.030,
                top_diameter=0.022,
                edge_radius=0.0007,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0007, width=0.0012),
                center=False,
            ),
            f"eq_knob_mesh_{index}",
        )
        for index in range(6)
    ]
    for index, ((x, y), mesh) in enumerate(zip(EQ_KNOB_POSITIONS, knob_meshes)):
        knob = model.part(f"eq_knob_{index}")
        knob.visual(mesh, origin=Origin(), material=KNOB_MAT, name="knob_body")
        knob.visual(
            Box((0.0025, 0.012, 0.0010)),
            origin=Origin(xyz=(0.0, 0.006, 0.0185)),
            material=WHITE_MAT,
            name="pointer_line",
        )
        model.articulation(
            f"housing_to_eq_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, y, PANEL_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check(
        "one housing plus twelve articulated controls",
        len(object_model.parts) == 13 and len(object_model.articulations) == 12,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    fader_specs = [
        ("crossfader", "housing_to_crossfader", "crossfader_slot", (1.0, 0.0, 0.0), -0.065, 0.065, "x"),
        *[
            (
                f"channel_fader_{index}",
                f"housing_to_channel_fader_{index}",
                f"channel_slot_{index}",
                (0.0, 1.0, 0.0),
                -0.052,
                0.052,
                "y",
            )
            for index in range(4)
        ],
        ("master_fader", "housing_to_master_fader", "master_slot", (0.0, 1.0, 0.0), -0.052, 0.052, "y"),
    ]
    for part_name, joint_name, slot_name, expected_axis, lower, upper, travel_axis in fader_specs:
        fader = object_model.get_part(part_name)
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{part_name} is a prismatic slot control",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == expected_axis
            and limits is not None
            and isclose(limits.lower, lower, abs_tol=1e-6)
            and isclose(limits.upper, upper, abs_tol=1e-6),
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_gap(
            fader,
            "housing",
            axis="z",
            positive_elem="slider_shoe",
            negative_elem=slot_name,
            max_gap=0.0002,
            max_penetration=0.00001,
            name=f"{part_name} shoe rides on its slot surface",
        )
        ctx.expect_overlap(
            fader,
            "housing",
            axes="xy",
            elem_a="slider_shoe",
            elem_b=slot_name,
            min_overlap=0.010,
            name=f"{part_name} is centered over its slot",
        )
        with ctx.pose({joint: upper}):
            ctx.expect_within(
                fader,
                "housing",
                axes=travel_axis,
                inner_elem="slider_shoe",
                outer_elem=slot_name,
                margin=0.0005,
                name=f"{part_name} remains inside slot at upper travel",
            )
        with ctx.pose({joint: lower}):
            ctx.expect_within(
                fader,
                "housing",
                axes=travel_axis,
                inner_elem="slider_shoe",
                outer_elem=slot_name,
                margin=0.0005,
                name=f"{part_name} remains inside slot at lower travel",
            )

    for index in range(6):
        knob = object_model.get_part(f"eq_knob_{index}")
        joint = object_model.get_articulation(f"housing_to_eq_knob_{index}")
        limits = joint.motion_limits
        ctx.check(
            f"eq_knob_{index} is a bounded rotary control",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.axis == (0.0, 0.0, 1.0)
            and limits is not None
            and isclose(limits.lower, -2.35, abs_tol=1e-6)
            and isclose(limits.upper, 2.35, abs_tol=1e-6),
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_gap(
            knob,
            "housing",
            axis="z",
            positive_elem="knob_body",
            negative_elem="front_panel_face",
            max_gap=0.0006,
            max_penetration=0.0,
            name=f"eq_knob_{index} mounts on the panel face",
        )

    return ctx.report()


object_model = build_object_model()

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


BRACKET_WIDTH = 0.240
BRACKET_DEPTH = 0.082
TOP_PLATE_THICKNESS = 0.016
CHEEK_THICKNESS = 0.014
CHEEK_DEPTH = 0.064
CHEEK_DROP = 0.112
INNER_SPAN = 0.170
TRUNNION_AXIS_Z = -0.058
TRUNNION_SHAFT_RADIUS = 0.012
BEARING_PAD_THICKNESS = 0.004
COLLAR_RADIUS = 0.019
COLLAR_THICKNESS = 0.005
UPPER_CHEEK_HEIGHT = 0.032
LOWER_CHEEK_HEIGHT = 0.034
LOWER_CHEEK_DEPTH = 0.038
REAR_WEB_DEPTH = 0.014
REAR_WEB_HEIGHT = 0.062

FACE_WIDTH = 0.148
FACE_THICKNESS = 0.016
FACE_HEIGHT = 0.140
FACE_CENTER_Z = -0.090


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_trunnion_module")

    bracket_finish = model.material("bracket_finish", rgba=(0.26, 0.29, 0.32, 1.0))
    carried_finish = model.material("carried_finish", rgba=(0.76, 0.79, 0.82, 1.0))

    support_bracket = model.part("support_bracket")
    cheek_center_x = INNER_SPAN / 2.0 + CHEEK_THICKNESS / 2.0
    upper_cheek_center_z = -TOP_PLATE_THICKNESS / 2.0 - UPPER_CHEEK_HEIGHT / 2.0
    lower_cheek_center_z = -TOP_PLATE_THICKNESS / 2.0 - CHEEK_DROP + LOWER_CHEEK_HEIGHT / 2.0
    rear_web_center_z = -TOP_PLATE_THICKNESS / 2.0 - CHEEK_DROP + REAR_WEB_HEIGHT / 2.0
    bearing_pad_center_x = INNER_SPAN / 2.0 + CHEEK_THICKNESS + BEARING_PAD_THICKNESS / 2.0

    support_bracket.visual(
        Box((BRACKET_WIDTH, BRACKET_DEPTH, TOP_PLATE_THICKNESS)),
        material=bracket_finish,
        name="top_plate",
    )
    support_bracket.visual(
        Box((0.094, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_THICKNESS / 2.0 + 0.006)),
        material=bracket_finish,
        name="mount_pad",
    )

    for sign, side in ((-1.0, "left"), (1.0, "right")):
        x = sign * cheek_center_x
        pad_x = sign * bearing_pad_center_x
        support_bracket.visual(
            Box((CHEEK_THICKNESS, CHEEK_DEPTH, UPPER_CHEEK_HEIGHT)),
            origin=Origin(xyz=(x, 0.0, upper_cheek_center_z)),
            material=bracket_finish,
            name=f"{side}_upper_cheek",
        )
        support_bracket.visual(
            Box((CHEEK_THICKNESS, LOWER_CHEEK_DEPTH, LOWER_CHEEK_HEIGHT)),
            origin=Origin(xyz=(x, 0.0, lower_cheek_center_z)),
            material=bracket_finish,
            name=f"{side}_lower_cheek",
        )
        support_bracket.visual(
            Box((CHEEK_THICKNESS, REAR_WEB_DEPTH, REAR_WEB_HEIGHT)),
            origin=Origin(xyz=(x, -0.025, rear_web_center_z)),
            material=bracket_finish,
            name=f"{side}_rear_web",
        )
        support_bracket.visual(
            Cylinder(radius=COLLAR_RADIUS, length=BEARING_PAD_THICKNESS),
            origin=Origin(
                xyz=(pad_x, 0.0, TRUNNION_AXIS_Z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=bracket_finish,
            name=f"{side}_bearing_pad",
        )

    support_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_WIDTH, BRACKET_DEPTH, CHEEK_DROP + TOP_PLATE_THICKNESS)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
    )

    carried_face = model.part("carried_face")
    shaft_length = INNER_SPAN + 2.0 * (CHEEK_THICKNESS + BEARING_PAD_THICKNESS + COLLAR_THICKNESS)
    collar_center_x = shaft_length / 2.0 - COLLAR_THICKNESS / 2.0

    carried_face.visual(
        Box((FACE_WIDTH, FACE_THICKNESS, FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FACE_CENTER_Z)),
        material=carried_finish,
        name="carried_face_panel",
    )
    carried_face.visual(
        Box((0.086, 0.030, 0.026)),
        material=carried_finish,
        name="axle_block",
    )
    carried_face.visual(
        Box((0.020, 0.024, 0.070)),
        origin=Origin(xyz=(-0.045, 0.0, -0.045)),
        material=carried_finish,
        name="left_hanger",
    )
    carried_face.visual(
        Box((0.020, 0.024, 0.070)),
        origin=Origin(xyz=(0.045, 0.0, -0.045)),
        material=carried_finish,
        name="right_hanger",
    )
    carried_face.visual(
        Cylinder(radius=TRUNNION_SHAFT_RADIUS, length=shaft_length),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=carried_finish,
        name="trunnion_shaft",
    )
    carried_face.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_THICKNESS),
        origin=Origin(
            xyz=(-collar_center_x, 0.0, 0.0),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=carried_finish,
        name="left_collar",
    )
    carried_face.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_THICKNESS),
        origin=Origin(
            xyz=(collar_center_x, 0.0, 0.0),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=carried_finish,
        name="right_collar",
    )
    carried_face.inertial = Inertial.from_geometry(
        Box((shaft_length, 0.040, 0.155)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    model.articulation(
        "trunnion_pitch",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=carried_face,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.70,
            upper=0.95,
            effort=40.0,
            velocity=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_bracket = object_model.get_part("support_bracket")
    carried_face = object_model.get_part("carried_face")
    trunnion_pitch = object_model.get_articulation("trunnion_pitch")

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

    ctx.expect_origin_gap(
        support_bracket,
        carried_face,
        axis="z",
        min_gap=0.050,
        max_gap=0.065,
        name="under-slung trunnion axis sits below the top bracket",
    )
    ctx.expect_origin_distance(
        support_bracket,
        carried_face,
        axes="x",
        max_dist=0.001,
        name="carried member stays centered on the cheek span",
    )
    ctx.expect_origin_distance(
        support_bracket,
        carried_face,
        axes="y",
        max_dist=0.001,
        name="carried member stays centered through the bracket depth",
    )
    ctx.expect_contact(
        carried_face,
        support_bracket,
        elem_a="left_collar",
        elem_b="left_bearing_pad",
        name="left trunnion collar seats against the left bearing pad",
    )
    ctx.expect_contact(
        carried_face,
        support_bracket,
        elem_a="right_collar",
        elem_b="right_bearing_pad",
        name="right trunnion collar seats against the right bearing pad",
    )

    rest_aabb = ctx.part_element_world_aabb(carried_face, elem="carried_face_panel")
    with ctx.pose({trunnion_pitch: 0.72}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pitched pose remains clear of the support bracket")
        pitched_aabb = ctx.part_element_world_aabb(carried_face, elem="carried_face_panel")

    def _center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    rest_center_y = _center_y(rest_aabb)
    pitched_center_y = _center_y(pitched_aabb)
    ctx.check(
        "positive pitch swings the carried face forward",
        rest_center_y is not None
        and pitched_center_y is not None
        and pitched_center_y > rest_center_y + 0.040,
        details=f"rest_center_y={rest_center_y}, pitched_center_y={pitched_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


OVERALL_WIDTH = 0.48
OVERALL_DEPTH = 0.25
BASE_THICKNESS = 0.018
BACK_THICKNESS = 0.014
OUTER_WALL_THICKNESS = 0.014
DIVIDER_THICKNESS = 0.012

BACK_HEIGHT = 0.26
SIDE_WALL_HEIGHT = 0.232
DIVIDER_HEIGHT = 0.214
FRONT_APRON_HEIGHT = 0.092

DIVIDER_X = 0.118

DRAWER_FACE_WIDTH = 0.068
DRAWER_FACE_HEIGHT = 0.078
DRAWER_FACE_THICKNESS = 0.014
DRAWER_DEPTH = 0.162
DRAWER_SIDE_THICKNESS = 0.008
DRAWER_BACK_THICKNESS = 0.008
DRAWER_BOTTOM_THICKNESS = 0.006
DRAWER_BOX_HEIGHT = 0.048
DRAWER_FRAME_Z = 0.056
DRAWER_TRAVEL = 0.09

GUIDE_WIDTH = 0.010
GUIDE_HEIGHT = 0.012
GUIDE_LENGTH = 0.214
GUIDE_CENTER_Y = 0.117
GUIDE_CENTER_Z = BASE_THICKNESS + GUIDE_HEIGHT * 0.5
GUIDE_OFFSET_X = 0.021

DRAWER_XS = {
    "left_drawer": -0.074,
    "center_drawer": 0.0,
    "right_drawer": 0.074,
}


def _add_drawer_geometry(part, *, wood_material, pull_material) -> None:
    side_length = DRAWER_DEPTH - DRAWER_BACK_THICKNESS
    side_center_y = side_length * 0.5
    bottom_length = DRAWER_DEPTH - 0.012
    bottom_center_y = 0.006 + bottom_length * 0.5

    part.visual(
        Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, DRAWER_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, -DRAWER_FACE_THICKNESS * 0.5, 0.0)),
        material=wood_material,
        name="front",
    )
    part.visual(
        Box((DRAWER_SIDE_THICKNESS, side_length, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DRAWER_FACE_WIDTH * 0.5 - DRAWER_SIDE_THICKNESS * 0.5),
                side_center_y,
                0.004,
            )
        ),
        material=wood_material,
        name="left_side",
    )
    part.visual(
        Box((DRAWER_SIDE_THICKNESS, side_length, DRAWER_BOX_HEIGHT)),
        origin=Origin(
            xyz=(
                DRAWER_FACE_WIDTH * 0.5 - DRAWER_SIDE_THICKNESS * 0.5,
                side_center_y,
                0.004,
            )
        ),
        material=wood_material,
        name="right_side",
    )
    part.visual(
        Box(
            (
                DRAWER_FACE_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_BACK_THICKNESS,
                DRAWER_BOX_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                DRAWER_DEPTH - DRAWER_BACK_THICKNESS * 0.5,
                0.004,
            )
        ),
        material=wood_material,
        name="back",
    )
    part.visual(
        Box(
            (
                DRAWER_FACE_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                bottom_length,
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                bottom_center_y,
                -0.023,
            )
        ),
        material=wood_material,
        name="bottom",
    )
    part.visual(
        Cylinder(radius=0.0065, length=0.024),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=pull_material,
        name="pull",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hardwood_tool_caddy")

    walnut = model.material("walnut", rgba=(0.46, 0.30, 0.18, 1.0))
    oak = model.material("oak", rgba=(0.70, 0.55, 0.34, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.24, 0.15, 0.09, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((OVERALL_WIDTH, OVERALL_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, OVERALL_DEPTH * 0.5, BASE_THICKNESS * 0.5)),
        material=walnut,
        name="base_panel",
    )
    carcass.visual(
        Box((OVERALL_WIDTH, BACK_THICKNESS, BACK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                OVERALL_DEPTH - BACK_THICKNESS * 0.5,
                BASE_THICKNESS + BACK_HEIGHT * 0.5,
            )
        ),
        material=walnut,
        name="back_panel",
    )
    carcass.visual(
        Box((OUTER_WALL_THICKNESS, OVERALL_DEPTH, SIDE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(OVERALL_WIDTH * 0.5 - OUTER_WALL_THICKNESS * 0.5),
                OVERALL_DEPTH * 0.5,
                BASE_THICKNESS + SIDE_WALL_HEIGHT * 0.5,
            )
        ),
        material=walnut,
        name="left_side_wall",
    )
    carcass.visual(
        Box((OUTER_WALL_THICKNESS, OVERALL_DEPTH, SIDE_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                OVERALL_WIDTH * 0.5 - OUTER_WALL_THICKNESS * 0.5,
                OVERALL_DEPTH * 0.5,
                BASE_THICKNESS + SIDE_WALL_HEIGHT * 0.5,
            )
        ),
        material=walnut,
        name="right_side_wall",
    )
    carcass.visual(
        Box((DIVIDER_THICKNESS, OVERALL_DEPTH, DIVIDER_HEIGHT)),
        origin=Origin(
            xyz=(
                -DIVIDER_X,
                OVERALL_DEPTH * 0.5,
                BASE_THICKNESS + DIVIDER_HEIGHT * 0.5,
            )
        ),
        material=oak,
        name="left_divider_wall",
    )
    carcass.visual(
        Box((DIVIDER_THICKNESS, OVERALL_DEPTH, DIVIDER_HEIGHT)),
        origin=Origin(
            xyz=(
                DIVIDER_X,
                OVERALL_DEPTH * 0.5,
                BASE_THICKNESS + DIVIDER_HEIGHT * 0.5,
            )
        ),
        material=oak,
        name="right_divider_wall",
    )
    carcass.visual(
        Box((0.102, DRAWER_FACE_THICKNESS, FRONT_APRON_HEIGHT)),
        origin=Origin(
            xyz=(
                -0.175,
                DRAWER_FACE_THICKNESS * 0.5,
                BASE_THICKNESS + FRONT_APRON_HEIGHT * 0.5,
            )
        ),
        material=walnut,
        name="left_front_apron",
    )
    carcass.visual(
        Box((0.102, DRAWER_FACE_THICKNESS, FRONT_APRON_HEIGHT)),
        origin=Origin(
            xyz=(
                0.175,
                DRAWER_FACE_THICKNESS * 0.5,
                BASE_THICKNESS + FRONT_APRON_HEIGHT * 0.5,
            )
        ),
        material=walnut,
        name="right_front_apron",
    )
    carcass.visual(
        Box((0.224, 0.130, 0.014)),
        origin=Origin(xyz=(0.0, 0.065, 0.103)),
        material=oak,
        name="center_shelf",
    )

    for drawer_name, drawer_x in DRAWER_XS.items():
        for side_name, sign in (("left", -1.0), ("right", 1.0)):
            carcass.visual(
                Box((GUIDE_WIDTH, GUIDE_LENGTH, GUIDE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        drawer_x + sign * GUIDE_OFFSET_X,
                        GUIDE_CENTER_Y,
                        GUIDE_CENTER_Z,
                    )
                ),
                material=oak,
                name=f"{drawer_name}_{side_name}_guide",
            )

    carcass.inertial = Inertial.from_geometry(
        Box((OVERALL_WIDTH, OVERALL_DEPTH, BACK_HEIGHT + BASE_THICKNESS)),
        mass=4.4,
        origin=Origin(
            xyz=(0.0, OVERALL_DEPTH * 0.5, (BACK_HEIGHT + BASE_THICKNESS) * 0.5)
        ),
    )

    for drawer_name, drawer_x in DRAWER_XS.items():
        drawer = model.part(drawer_name)
        _add_drawer_geometry(drawer, wood_material=oak, pull_material=dark_wood)
        drawer.inertial = Inertial.from_geometry(
            Box(
                (
                    DRAWER_FACE_WIDTH,
                    DRAWER_DEPTH + DRAWER_FACE_THICKNESS,
                    DRAWER_FACE_HEIGHT,
                )
            ),
            mass=0.35,
            origin=Origin(
                xyz=(
                    0.0,
                    (DRAWER_DEPTH - DRAWER_FACE_THICKNESS) * 0.5,
                    0.0,
                )
            ),
        )
        model.articulation(
            f"carcass_to_{drawer_name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(drawer_x, 0.0, DRAWER_FRAME_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.20,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
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

    carcass = object_model.get_part("carcass")

    for drawer_name in DRAWER_XS:
        drawer = object_model.get_part(drawer_name)
        slide_joint = object_model.get_articulation(f"carcass_to_{drawer_name}")

        ctx.expect_contact(
            drawer,
            carcass,
            elem_a="bottom",
            elem_b=f"{drawer_name}_left_guide",
            name=f"{drawer_name} bottom rests on left wooden guide",
        )
        ctx.expect_contact(
            drawer,
            carcass,
            elem_a="bottom",
            elem_b=f"{drawer_name}_right_guide",
            name=f"{drawer_name} bottom rests on right wooden guide",
        )

        rest_pos = ctx.part_world_position(drawer)
        upper = slide_joint.motion_limits.upper if slide_joint.motion_limits else 0.0

        with ctx.pose({slide_joint: upper}):
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a="bottom",
                elem_b=f"{drawer_name}_left_guide",
                name=f"{drawer_name} stays supported on left guide when opened",
            )
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a="bottom",
                elem_b=f"{drawer_name}_right_guide",
                name=f"{drawer_name} stays supported on right guide when opened",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="bottom",
                elem_b=f"{drawer_name}_left_guide",
                min_overlap=0.05,
                name=f"{drawer_name} keeps retained insertion on its bottom guide",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{drawer_name} slides outward from the front face",
            rest_pos is not None
            and open_pos is not None
            and open_pos[1] < rest_pos[1] - 0.08,
            details=f"rest={rest_pos}, open={open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

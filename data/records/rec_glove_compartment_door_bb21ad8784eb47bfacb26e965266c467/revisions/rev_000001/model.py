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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rounded_panel_with_hole_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    hole_width: float,
    hole_height: float,
    hole_center_y: float,
    hole_corner_radius: float,
    mesh_name: str,
):
    outer_profile = rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    hole_profile = _shift_profile(
        rounded_rect_profile(
            hole_width,
            hole_height,
            hole_corner_radius,
            corner_segments=6,
        ),
        dy=hole_center_y,
    )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [hole_profile],
            thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        mesh_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_sedan_glove_box")

    dash_plastic = model.material("dash_plastic", rgba=(0.14, 0.15, 0.17, 1.0))
    inner_bin_plastic = model.material("inner_bin_plastic", rgba=(0.18, 0.19, 0.22, 1.0))
    hinge_plastic = model.material("hinge_plastic", rgba=(0.11, 0.12, 0.14, 1.0))
    woodgrain = model.material("woodgrain", rgba=(0.45, 0.29, 0.16, 1.0))
    button_trim = model.material("button_trim", rgba=(0.69, 0.70, 0.72, 1.0))
    strap_metal = model.material("strap_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    strap_bushing = model.material("strap_bushing", rgba=(0.09, 0.10, 0.11, 1.0))

    opening_width = 0.384
    opening_height = 0.168
    bin_depth = 0.168
    wall = 0.004
    shell_width = opening_width + 0.024
    shell_height = opening_height + 0.016
    door_width = 0.374
    door_height = 0.164
    door_body_thickness = 0.018
    button_width = 0.020
    button_height = 0.010
    button_travel = 0.006
    button_z = -0.042

    bezel_mesh = _rounded_panel_with_hole_mesh(
        width=0.430,
        height=0.208,
        thickness=0.004,
        corner_radius=0.018,
        hole_width=opening_width,
        hole_height=opening_height,
        hole_center_y=0.0,
        hole_corner_radius=0.012,
        mesh_name="glovebox_opening_bezel",
    )
    door_body_mesh = _rounded_panel_with_hole_mesh(
        width=door_width,
        height=door_height,
        thickness=door_body_thickness,
        corner_radius=0.014,
        hole_width=0.024,
        hole_height=0.012,
        hole_center_y=0.040,
        hole_corner_radius=0.0035,
        mesh_name="glovebox_door_body",
    )
    veneer_mesh = _rounded_panel_with_hole_mesh(
        width=0.344,
        height=0.124,
        thickness=0.0025,
        corner_radius=0.010,
        hole_width=0.022,
        hole_height=0.011,
        hole_center_y=0.029,
        hole_corner_radius=0.003,
        mesh_name="glovebox_door_veneer",
    )

    glovebox_bin = model.part("glovebox_bin")
    glovebox_bin.visual(
        Box((shell_width, bin_depth, wall)),
        origin=Origin(xyz=(0.0, -bin_depth / 2.0, -wall / 2.0)),
        material=inner_bin_plastic,
        name="bin_floor",
    )
    glovebox_bin.visual(
        Box((shell_width, bin_depth, wall)),
        origin=Origin(xyz=(0.0, -bin_depth / 2.0, opening_height + wall / 2.0)),
        material=inner_bin_plastic,
        name="bin_roof",
    )
    glovebox_bin.visual(
        Box((wall, bin_depth, shell_height)),
        origin=Origin(
            xyz=(-shell_width / 2.0 + wall / 2.0, -bin_depth / 2.0, (opening_height - wall) / 2.0)
        ),
        material=inner_bin_plastic,
        name="left_bin_wall",
    )
    glovebox_bin.visual(
        Box((wall, bin_depth, shell_height)),
        origin=Origin(
            xyz=(shell_width / 2.0 - wall / 2.0, -bin_depth / 2.0, (opening_height - wall) / 2.0)
        ),
        material=inner_bin_plastic,
        name="right_bin_wall",
    )
    glovebox_bin.visual(
        Box((shell_width, wall, shell_height)),
        origin=Origin(xyz=(0.0, -bin_depth + wall / 2.0, (opening_height - wall) / 2.0)),
        material=inner_bin_plastic,
        name="bin_back_wall",
    )
    glovebox_bin.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, -0.002, opening_height / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dash_plastic,
        name="opening_bezel",
    )
    glovebox_bin.inertial = Inertial.from_geometry(
        Box((shell_width, bin_depth, shell_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -bin_depth / 2.0, opening_height / 2.0)),
    )

    glovebox_door = model.part("glovebox_door")
    glovebox_door.visual(
        door_body_mesh,
        origin=Origin(xyz=(0.0, -0.007, -0.094), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dash_plastic,
        name="door_body",
    )
    glovebox_door.visual(
        veneer_mesh,
        origin=Origin(xyz=(0.0, 0.00325, -0.094), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=woodgrain,
        name="woodgrain_insert",
    )
    glovebox_door.visual(
        Box((0.020, 0.008, 0.018)),
        origin=Origin(xyz=(-door_width / 2.0 + 0.018, -0.017, -0.096)),
        material=hinge_plastic,
        name="left_strap_tab",
    )
    glovebox_door.visual(
        Box((0.020, 0.008, 0.018)),
        origin=Origin(xyz=(door_width / 2.0 - 0.018, -0.017, -0.096)),
        material=hinge_plastic,
        name="right_strap_tab",
    )
    glovebox_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.024, door_height)),
        mass=1.05,
        origin=Origin(xyz=(0.0, -0.007, -0.094)),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((button_width, 0.004, button_height)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=button_trim,
        name="button_cap",
    )
    latch_button.visual(
        Box((0.010, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=strap_bushing,
        name="button_stem",
    )
    latch_button.inertial = Inertial.from_geometry(
        Box((button_width, 0.016, button_height)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )

    def _add_check_strap(name: str) -> None:
        strap = model.part(name)
        strap.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=strap_bushing,
            name="strap_pivot",
        )
        strap.visual(
            Box((0.0035, 0.008, 0.096)),
            origin=Origin(xyz=(0.0, -0.015, -0.048), rpy=(-0.24, 0.0, 0.0)),
            material=strap_metal,
            name="strap_body",
        )
        strap.visual(
            Cylinder(radius=0.0045, length=0.010),
            origin=Origin(xyz=(0.0, -0.029, -0.092), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=strap_bushing,
            name="strap_tip",
        )
        strap.inertial = Inertial.from_geometry(
            Box((0.008, 0.034, 0.100)),
            mass=0.045,
            origin=Origin(xyz=(0.0, -0.016, -0.050)),
        )

    _add_check_strap("left_check_strap")
    _add_check_strap("right_check_strap")

    model.articulation(
        "bin_to_door",
        ArticulationType.REVOLUTE,
        parent=glovebox_bin,
        child=glovebox_door,
        origin=Origin(xyz=(0.0, 0.016, opening_height + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "door_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=glovebox_door,
        child=latch_button,
        origin=Origin(xyz=(0.0, -0.006, button_z - 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.10,
            lower=0.0,
            upper=button_travel,
        ),
    )
    model.articulation(
        "bin_to_left_check_strap",
        ArticulationType.REVOLUTE,
        parent=glovebox_bin,
        child=model.get_part("left_check_strap"),
        origin=Origin(xyz=(-shell_width / 2.0 + wall + 0.004, -0.026, opening_height - 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "bin_to_right_check_strap",
        ArticulationType.REVOLUTE,
        parent=glovebox_bin,
        child=model.get_part("right_check_strap"),
        origin=Origin(xyz=(shell_width / 2.0 - wall - 0.004, -0.026, opening_height - 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
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

    glovebox_bin = object_model.get_part("glovebox_bin")
    glovebox_door = object_model.get_part("glovebox_door")
    latch_button = object_model.get_part("latch_button")
    left_check_strap = object_model.get_part("left_check_strap")
    right_check_strap = object_model.get_part("right_check_strap")

    door_hinge = object_model.get_articulation("bin_to_door")
    button_slide = object_model.get_articulation("door_to_latch_button")
    left_strap_joint = object_model.get_articulation("bin_to_left_check_strap")
    right_strap_joint = object_model.get_articulation("bin_to_right_check_strap")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    ctx.expect_gap(
        glovebox_door,
        glovebox_bin,
        axis="y",
        positive_elem="door_body",
        negative_elem="opening_bezel",
        max_penetration=0.00001,
        max_gap=0.0035,
        name="door closes flush against the dash bezel",
    )
    ctx.expect_overlap(
        glovebox_door,
        glovebox_bin,
        axes="xz",
        elem_a="door_body",
        elem_b="opening_bezel",
        min_overlap=0.160,
        name="door broadly covers the glove box opening",
    )

    closed_lower = _aabb_center(ctx.part_element_world_aabb(glovebox_door, elem="door_body"))
    with ctx.pose({door_hinge: math.radians(72.0)}):
        open_lower = _aabb_center(ctx.part_element_world_aabb(glovebox_door, elem="door_body"))
    ctx.check(
        "door opens upward into the dash cavity",
        closed_lower is not None
        and open_lower is not None
        and open_lower[2] > closed_lower[2] + 0.030
        and open_lower[1] < closed_lower[1] - 0.055,
        details=f"closed_lower={closed_lower}, open_lower={open_lower}",
    )

    rest_button = _aabb_center(ctx.part_element_world_aabb(latch_button, elem="button_cap"))
    with ctx.pose({button_slide: button_slide.motion_limits.upper}):
        pressed_button = _aabb_center(ctx.part_element_world_aabb(latch_button, elem="button_cap"))
    ctx.check(
        "latch button depresses into the door face",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] < rest_button[1] - 0.0045,
        details=f"rest_button={rest_button}, pressed_button={pressed_button}",
    )

    left_tip_rest = _aabb_center(ctx.part_element_world_aabb(left_check_strap, elem="strap_tip"))
    right_tip_rest = _aabb_center(ctx.part_element_world_aabb(right_check_strap, elem="strap_tip"))
    with ctx.pose(
        {
            door_hinge: math.radians(62.0),
            left_strap_joint: math.radians(42.0),
            right_strap_joint: math.radians(42.0),
        }
    ):
        left_tip_open = _aabb_center(ctx.part_element_world_aabb(left_check_strap, elem="strap_tip"))
        right_tip_open = _aabb_center(ctx.part_element_world_aabb(right_check_strap, elem="strap_tip"))
    ctx.check(
        "left check strap swings upward on its support pivot",
        left_tip_rest is not None
        and left_tip_open is not None
        and left_tip_open[2] > left_tip_rest[2] + 0.030
        and left_tip_open[1] < left_tip_rest[1] - 0.030,
        details=f"left_tip_rest={left_tip_rest}, left_tip_open={left_tip_open}",
    )
    ctx.check(
        "right check strap swings upward on its support pivot",
        right_tip_rest is not None
        and right_tip_open is not None
        and right_tip_open[2] > right_tip_rest[2] + 0.030
        and right_tip_open[1] < right_tip_rest[1] - 0.030,
        details=f"right_tip_rest={right_tip_rest}, right_tip_open={right_tip_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

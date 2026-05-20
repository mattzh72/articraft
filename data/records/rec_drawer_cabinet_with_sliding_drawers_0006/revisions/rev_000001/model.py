from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import tempfile
import math

_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd():
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        fallback = tempfile.gettempdir()
        os.chdir(fallback)
        return fallback


os.getcwd = _safe_getcwd
os.chdir(os.getcwd())

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSET_ROOT = tempfile.gettempdir()


def _ensure_valid_cwd():
    try:
        os.getcwd()
    except FileNotFoundError:
        os.chdir(ASSET_ROOT)

CABINET_WIDTH = 0.90
CABINET_DEPTH = 0.56
CARCASS_HEIGHT = 0.72
COUNTER_THICKNESS = 0.038
COUNTER_WIDTH = 0.94
COUNTER_DEPTH = 0.60
COUNTER_CENTER_Y = -0.27

SIDE_THICKNESS = 0.018
BACK_THICKNESS = 0.006
BOTTOM_THICKNESS = 0.018
KICK_HEIGHT = 0.10
KICK_RECESS = 0.075

DRAWER_FRONT_THICKNESS = 0.020
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.008
SLIDE_WIDTH = 0.020
DRAWER_SLIDE_WIDTH = 0.010
SLIDE_HEIGHT = 0.030
SLIDE_FRONT_SETBACK = 0.020
SLIDE_LENGTH = 0.500
DRAWER_FRONT_WIDTH = 0.856
DRAWER_BOX_WIDTH = 0.804
DRAWER_SLIDE_CENTER = 0.407
CARCASS_SLIDE_CENTER = 0.422

DRAWER_SPECS = (
    {
        "name": "bottom_drawer",
        "front_height": 0.240,
        "bottom_z": 0.122,
        "box_height": 0.170,
        "box_bottom_offset": 0.032,
        "depth": 0.500,
        "travel": 0.500,
        "carcass_left_slide": "slide_bottom_left",
        "carcass_right_slide": "slide_bottom_right",
    },
    {
        "name": "middle_drawer",
        "front_height": 0.190,
        "bottom_z": 0.366,
        "box_height": 0.125,
        "box_bottom_offset": 0.028,
        "depth": 0.420,
        "travel": 0.420,
        "carcass_left_slide": "slide_middle_left",
        "carcass_right_slide": "slide_middle_right",
    },
    {
        "name": "top_drawer",
        "front_height": 0.150,
        "bottom_z": 0.560,
        "box_height": 0.090,
        "box_bottom_offset": 0.026,
        "depth": 0.320,
        "travel": 0.320,
        "carcass_left_slide": "slide_top_left",
        "carcass_right_slide": "slide_top_right",
    },
)


def _add_box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _drawer_slide_center_z(spec):
    return spec["box_bottom_offset"] + (spec["box_height"] / 2.0)


def build_object_model() -> ArticulatedObject:
    _ensure_valid_cwd()
    model = ArticulatedObject(name="kitchen_base_cabinet")

    oak = model.material("oak", rgba=(0.73, 0.57, 0.37, 1.0))
    birch = model.material("birch_ply", rgba=(0.84, 0.77, 0.62, 1.0))
    steel = model.material("slide_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    stone = model.material("counter_stone", rgba=(0.67, 0.67, 0.64, 1.0))

    carcass = model.part("carcass")
    _add_box(
        carcass,
        (SIDE_THICKNESS, CABINET_DEPTH, CARCASS_HEIGHT),
        (-(CABINET_WIDTH / 2.0) + (SIDE_THICKNESS / 2.0), -(CABINET_DEPTH / 2.0), CARCASS_HEIGHT / 2.0),
        oak,
        "left_panel",
    )
    _add_box(
        carcass,
        (SIDE_THICKNESS, CABINET_DEPTH, CARCASS_HEIGHT),
        ((CABINET_WIDTH / 2.0) - (SIDE_THICKNESS / 2.0), -(CABINET_DEPTH / 2.0), CARCASS_HEIGHT / 2.0),
        oak,
        "right_panel",
    )
    _add_box(
        carcass,
        (CABINET_WIDTH - (2.0 * SIDE_THICKNESS), CABINET_DEPTH, BOTTOM_THICKNESS),
        (0.0, -(CABINET_DEPTH / 2.0), KICK_HEIGHT + (BOTTOM_THICKNESS / 2.0)),
        oak,
        "bottom_panel",
    )
    _add_box(
        carcass,
        (CABINET_WIDTH - (2.0 * SIDE_THICKNESS), BACK_THICKNESS, CARCASS_HEIGHT - KICK_HEIGHT),
        (0.0, -CABINET_DEPTH + (BACK_THICKNESS / 2.0), KICK_HEIGHT + ((CARCASS_HEIGHT - KICK_HEIGHT) / 2.0)),
        oak,
        "back_panel",
    )
    _add_box(
        carcass,
        (CABINET_WIDTH - (2.0 * SIDE_THICKNESS), CABINET_DEPTH - KICK_RECESS, KICK_HEIGHT),
        (0.0, -((CABINET_DEPTH + KICK_RECESS) / 2.0), KICK_HEIGHT / 2.0),
        oak,
        "plinth_block",
    )
    _add_box(
        carcass,
        (CABINET_WIDTH - (2.0 * SIDE_THICKNESS), 0.012, KICK_HEIGHT),
        (0.0, -KICK_RECESS - 0.006, KICK_HEIGHT / 2.0),
        oak,
        "toe_kick_panel",
    )
    _add_box(
        carcass,
        (CABINET_WIDTH - (2.0 * SIDE_THICKNESS), 0.060, SIDE_THICKNESS),
        (0.0, -CABINET_DEPTH + 0.030, CARCASS_HEIGHT - (SIDE_THICKNESS / 2.0)),
        oak,
        "rear_stretcher",
    )

    for spec in DRAWER_SPECS:
        slide_z = spec["bottom_z"] + _drawer_slide_center_z(spec)
        for side_sign, visual_name in (
            (-1.0, spec["carcass_left_slide"]),
            (1.0, spec["carcass_right_slide"]),
        ):
            _add_box(
                carcass,
                (SLIDE_WIDTH, SLIDE_LENGTH, SLIDE_HEIGHT),
                (
                    side_sign * CARCASS_SLIDE_CENTER,
                    -SLIDE_FRONT_SETBACK - (SLIDE_LENGTH / 2.0),
                    slide_z,
                ),
                steel,
                visual_name,
            )

    countertop = model.part("countertop")
    _add_box(
        countertop,
        (COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS),
        (0.0, COUNTER_CENTER_Y, COUNTER_THICKNESS / 2.0),
        stone,
        "counter_slab",
    )
    model.articulation(
        "carcass_to_countertop",
        ArticulationType.FIXED,
        parent=carcass,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, CARCASS_HEIGHT)),
    )

    for spec in DRAWER_SPECS:
        drawer = model.part(spec["name"])
        front_height = spec["front_height"]
        box_height = spec["box_height"]
        box_bottom = spec["box_bottom_offset"]
        depth = spec["depth"]
        side_length = depth - DRAWER_FRONT_THICKNESS
        side_center_y = -((depth + DRAWER_FRONT_THICKNESS) / 2.0)

        _add_box(
            drawer,
            (DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, front_height),
            (0.0, -(DRAWER_FRONT_THICKNESS / 2.0), front_height / 2.0),
            oak,
            "front",
        )
        pull_span = min(0.24, DRAWER_FRONT_WIDTH * 0.34)
        pull_z = front_height * 0.52
        for pull_x, post_name in ((-0.060, "left_pull_post"), (0.060, "right_pull_post")):
            drawer.visual(
                Cylinder(radius=0.0045, length=0.018),
                origin=Origin(xyz=(pull_x, 0.009, pull_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=post_name,
            )
        drawer.visual(
            Cylinder(radius=0.005, length=pull_span),
            origin=Origin(xyz=(0.0, 0.018, pull_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="pull_bar",
        )
        _add_box(
            drawer,
            (DRAWER_SIDE_THICKNESS, side_length, box_height),
            (-(DRAWER_BOX_WIDTH / 2.0) + (DRAWER_SIDE_THICKNESS / 2.0), side_center_y, box_bottom + (box_height / 2.0)),
            birch,
            "left_side",
        )
        _add_box(
            drawer,
            (DRAWER_SIDE_THICKNESS, side_length, box_height),
            ((DRAWER_BOX_WIDTH / 2.0) - (DRAWER_SIDE_THICKNESS / 2.0), side_center_y, box_bottom + (box_height / 2.0)),
            birch,
            "right_side",
        )
        _add_box(
            drawer,
            (DRAWER_BOX_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS), side_length, DRAWER_BOTTOM_THICKNESS),
            (0.0, side_center_y, box_bottom + (DRAWER_BOTTOM_THICKNESS / 2.0)),
            birch,
            "bottom",
        )
        _add_box(
            drawer,
            (DRAWER_BOX_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS), DRAWER_SIDE_THICKNESS, box_height),
            (0.0, -depth + (DRAWER_SIDE_THICKNESS / 2.0), box_bottom + (box_height / 2.0)),
            birch,
            "back",
        )
        _add_box(
            drawer,
            (DRAWER_SLIDE_WIDTH, max(depth - 0.020, 0.020), SLIDE_HEIGHT),
            (-DRAWER_SLIDE_CENTER, -(depth / 2.0), box_bottom + (box_height / 2.0)),
            steel,
            "left_slide",
        )
        _add_box(
            drawer,
            (DRAWER_SLIDE_WIDTH, max(depth - 0.020, 0.020), SLIDE_HEIGHT),
            (DRAWER_SLIDE_CENTER, -(depth / 2.0), box_bottom + (box_height / 2.0)),
            steel,
            "right_slide",
        )
        model.articulation(
            f"carcass_to_{spec['name']}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, spec["bottom_z"])),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.60,
                lower=0.0,
                upper=spec["travel"],
            ),
        )

    return model


def run_tests() -> TestReport:
    _ensure_valid_cwd()
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)
    carcass = object_model.get_part("carcass")
    countertop = object_model.get_part("countertop")
    counter_slab = countertop.get_visual("counter_slab")
    carcass_back = carcass.get_visual("back_panel")
    carcass_left = carcass.get_visual("left_panel")
    carcass_right = carcass.get_visual("right_panel")
    toe_kick_panel = carcass.get_visual("toe_kick_panel")
    top_drawer = object_model.get_part("top_drawer")
    middle_drawer = object_model.get_part("middle_drawer")
    bottom_drawer = object_model.get_part("bottom_drawer")
    top_slide = object_model.get_articulation("carcass_to_top_drawer")
    middle_slide = object_model.get_articulation("carcass_to_middle_drawer")
    bottom_slide = object_model.get_articulation("carcass_to_bottom_drawer")
    top_front = top_drawer.get_visual("front")
    middle_front = middle_drawer.get_visual("front")
    bottom_front = bottom_drawer.get_visual("front")
    top_back = top_drawer.get_visual("back")
    middle_back = middle_drawer.get_visual("back")
    bottom_back = bottom_drawer.get_visual("back")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(countertop, carcass, elem_a=counter_slab)
    ctx.expect_overlap(countertop, carcass, axes="xy", min_overlap=0.45)
    ctx.expect_gap(countertop, carcass, axis="z", max_gap=0.001, max_penetration=0.0)

    ctx.expect_gap(top_drawer, middle_drawer, axis="z", min_gap=0.003, max_gap=0.005, positive_elem=top_front, negative_elem=middle_front)
    ctx.expect_gap(middle_drawer, bottom_drawer, axis="z", min_gap=0.003, max_gap=0.005, positive_elem=middle_front, negative_elem=bottom_front)
    ctx.expect_gap(countertop, top_drawer, axis="z", min_gap=0.009, max_gap=0.011, positive_elem=counter_slab, negative_elem=top_front)
    ctx.expect_gap(bottom_drawer, carcass, axis="z", min_gap=0.021, max_gap=0.023, positive_elem=bottom_front, negative_elem=toe_kick_panel)

    ctx.expect_gap(top_drawer, carcass, axis="y", min_gap=0.232, max_gap=0.236, positive_elem=top_back, negative_elem=carcass_back)
    ctx.expect_gap(middle_drawer, carcass, axis="y", min_gap=0.132, max_gap=0.136, positive_elem=middle_back, negative_elem=carcass_back)
    ctx.expect_gap(bottom_drawer, carcass, axis="y", min_gap=0.052, max_gap=0.056, positive_elem=bottom_back, negative_elem=carcass_back)

    ctx.check(
        "drawer_depths_are_graduated",
        DRAWER_SPECS[0]["depth"] > DRAWER_SPECS[1]["depth"] > DRAWER_SPECS[2]["depth"],
        details="Drawer box depths should step shallower from bottom to top.",
    )

    for spec in DRAWER_SPECS:
        drawer = object_model.get_part(spec["name"])
        joint = object_model.get_articulation(f"carcass_to_{spec['name']}")
        front = drawer.get_visual("front")
        back = drawer.get_visual("back")
        left_slide = drawer.get_visual("left_slide")
        right_slide = drawer.get_visual("right_slide")
        left_post = drawer.get_visual("left_pull_post")
        right_post = drawer.get_visual("right_pull_post")
        pull_bar = drawer.get_visual("pull_bar")
        limits = joint.motion_limits

        ctx.check(
            f"{spec['name']}_slides_along_y",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"{joint.name} axis was {joint.axis!r}, expected a front-to-back slide along +Y.",
        )
        ctx.check(
            f"{spec['name']}_travel_matches_box_depth",
            limits is not None and limits.lower == 0.0 and limits.upper == spec["travel"],
            details=f"{joint.name} limits should be 0.0 to {spec['travel']:.3f} m for full extension.",
        )

        ctx.expect_contact(
            drawer,
            carcass,
            elem_a=left_slide,
            elem_b=carcass.get_visual(spec["carcass_left_slide"]),
            name=f"{spec['name']}_left_slide_contact",
        )
        ctx.expect_contact(
            drawer,
            carcass,
            elem_a=right_slide,
            elem_b=carcass.get_visual(spec["carcass_right_slide"]),
            name=f"{spec['name']}_right_slide_contact",
        )
        ctx.expect_contact(
            drawer,
            drawer,
            elem_a=left_post,
            elem_b=pull_bar,
            name=f"{spec['name']}_left_pull_connected",
        )
        ctx.expect_contact(
            drawer,
            drawer,
            elem_a=right_post,
            elem_b=pull_bar,
            name=f"{spec['name']}_right_pull_connected",
        )
        ctx.expect_within(
            drawer,
            carcass,
            axes="x",
            inner_elem=front,
            margin=0.0,
            name=f"{spec['name']}_front_within_case_width",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="x",
            min_overlap=0.84,
            elem_a=front,
            name=f"{spec['name']}_front_aligned_to_opening",
        )

        closed_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"{spec['name']}_closed_position_available",
            closed_pos is not None,
            details="Drawer world position should be measurable in the rest pose.",
        )

        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{spec['name']}_closed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{spec['name']}_closed_no_floating")

            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{spec['name']}_open_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{spec['name']}_open_no_floating")
                ctx.expect_gap(
                    drawer,
                    carcass,
                    axis="y",
                    min_gap=-0.001,
                    max_gap=0.002,
                    positive_elem=back,
                    negative_elem=carcass_left,
                    name=f"{spec['name']}_full_extension_reaches_front_plane",
                )

                open_pos = ctx.part_world_position(drawer)
                ctx.check(
                    f"{spec['name']}_full_extension_distance",
                    open_pos is not None
                    and closed_pos is not None
                    and abs((open_pos[1] - closed_pos[1]) - spec["travel"]) <= 1e-6,
                    details=f"{spec['name']} should translate by {spec['travel']:.3f} m at full extension.",
                )

    with ctx.pose({top_slide: 0.320, middle_slide: 0.420, bottom_slide: 0.500}):
        ctx.expect_gap(top_drawer, carcass, axis="y", min_gap=-0.001, max_gap=0.002, positive_elem=top_back, negative_elem=carcass_left, name="top_drawer_reaches_full_extension")
        ctx.expect_gap(middle_drawer, carcass, axis="y", min_gap=-0.001, max_gap=0.002, positive_elem=middle_back, negative_elem=carcass_left, name="middle_drawer_reaches_full_extension")
        ctx.expect_gap(bottom_drawer, carcass, axis="y", min_gap=-0.001, max_gap=0.002, positive_elem=bottom_back, negative_elem=carcass_left, name="bottom_drawer_reaches_full_extension")
        ctx.expect_gap(countertop, top_drawer, axis="z", min_gap=0.009, max_gap=0.011, positive_elem=counter_slab, negative_elem=top_front)
        ctx.expect_within(top_drawer, carcass, axes="x", inner_elem=top_front, margin=0.0, name="top_drawer_stays_centered_when_open")

    return ctx.report()


# >>> USER_CODE_END

_ensure_valid_cwd()
object_model = build_object_model()

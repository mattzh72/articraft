from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_dog_door")

    frame_white = model.material("frame_white", rgba=(0.93, 0.93, 0.90, 1.0))
    sleeve_gray = model.material("sleeve_gray", rgba=(0.72, 0.72, 0.72, 1.0))
    flap_smoke = model.material("flap_smoke", rgba=(0.18, 0.22, 0.26, 0.60))
    latch_black = model.material("latch_black", rgba=(0.12, 0.12, 0.12, 1.0))

    outer_w = 0.50
    outer_h = 0.76
    frame_depth = 0.11
    opening_w = 0.38
    opening_h = 0.60

    side_border = (outer_w - opening_w) / 2.0
    top_border = (outer_h - opening_h) / 2.0
    trim_depth = 0.018
    tunnel_wall = 0.014
    tunnel_depth = 0.082
    front_y = frame_depth / 2.0 - trim_depth / 2.0
    back_y = -front_y
    hinge_y = 0.023
    hinge_z = opening_h / 2.0 - 0.015

    frame = model.part("frame")

    def add_trim_ring(prefix: str, y_center: float) -> None:
        frame.visual(
            Box((side_border, trim_depth, outer_h)),
            origin=Origin(xyz=(-(opening_w / 2.0 + side_border / 2.0), y_center, 0.0)),
            material=frame_white,
            name=f"{prefix}_left_jamb",
        )
        frame.visual(
            Box((side_border, trim_depth, outer_h)),
            origin=Origin(xyz=((opening_w / 2.0 + side_border / 2.0), y_center, 0.0)),
            material=frame_white,
            name=f"{prefix}_right_jamb",
        )
        frame.visual(
            Box((outer_w, trim_depth, top_border)),
            origin=Origin(xyz=(0.0, y_center, opening_h / 2.0 + top_border / 2.0)),
            material=frame_white,
            name=f"{prefix}_header",
        )
        frame.visual(
            Box((outer_w, trim_depth, top_border)),
            origin=Origin(xyz=(0.0, y_center, -(opening_h / 2.0 + top_border / 2.0))),
            material=frame_white,
            name=f"{prefix}_sill",
        )

    add_trim_ring("front_trim", front_y)
    add_trim_ring("back_trim", back_y)

    frame.visual(
        Box((tunnel_wall, tunnel_depth, opening_h)),
        origin=Origin(xyz=(-(opening_w / 2.0 + tunnel_wall / 2.0), 0.0, 0.0)),
        material=sleeve_gray,
        name="left_tunnel_jamb",
    )
    frame.visual(
        Box((tunnel_wall, tunnel_depth, opening_h)),
        origin=Origin(xyz=((opening_w / 2.0 + tunnel_wall / 2.0), 0.0, 0.0)),
        material=sleeve_gray,
        name="right_tunnel_jamb",
    )
    frame.visual(
        Box((opening_w + 2.0 * tunnel_wall, tunnel_depth, tunnel_wall)),
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0 + tunnel_wall / 2.0)),
        material=sleeve_gray,
        name="top_tunnel_header",
    )
    frame.visual(
        Box((opening_w + 2.0 * tunnel_wall, tunnel_depth, tunnel_wall)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_h / 2.0 + tunnel_wall / 2.0))),
        material=sleeve_gray,
        name="bottom_tunnel_sill",
    )
    frame.visual(
        Box((0.37, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z + 0.009)),
        material=latch_black,
        name="hinge_carrier",
    )

    flap = model.part("flap")
    flap_width = 0.35
    flap_height = 0.57
    flap_thickness = 0.010

    flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, 0.0, -flap_height / 2.0)),
        material=flap_smoke,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_width, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=latch_black,
        name="flap_top_rail",
    )
    flap.visual(
        Box((0.10, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, -0.008, -0.528)),
        material=latch_black,
        name="latch_mount",
    )

    latch_tab = model.part("latch_tab")
    latch_tab.visual(
        Box((0.075, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=latch_black,
        name="tab_plate",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "flap_to_latch_tab",
        ArticulationType.REVOLUTE,
        parent=flap,
        child=latch_tab,
        origin=Origin(xyz=(0.0, -0.010, -0.542)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.90,
            upper=0.25,
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

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    latch_tab = object_model.get_part("latch_tab")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    latch_pivot = object_model.get_articulation("flap_to_latch_tab")

    with ctx.pose({flap_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            flap,
            frame,
            axis="x",
            positive_elem="flap_panel",
            negative_elem="left_tunnel_jamb",
            min_gap=0.010,
            max_gap=0.030,
            name="flap clears left side of opening",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="x",
            positive_elem="right_tunnel_jamb",
            negative_elem="flap_panel",
            min_gap=0.010,
            max_gap=0.030,
            name="flap clears right side of opening",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            positive_elem="top_tunnel_header",
            negative_elem="flap_panel",
            min_gap=0.005,
            max_gap=0.030,
            name="flap hangs just below upper tunnel header",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="bottom_tunnel_sill",
            min_gap=0.005,
            max_gap=0.030,
            name="flap clears lower tunnel sill",
        )
        ctx.expect_contact(
            frame,
            flap,
            elem_a="hinge_carrier",
            elem_b="flap_top_rail",
            contact_tol=0.001,
            name="flap is physically supported by the top hinge carrier",
        )
        ctx.expect_contact(
            flap,
            latch_tab,
            elem_a="latch_mount",
            elem_b="tab_plate",
            contact_tol=0.0015,
            name="latch tab is mounted to the bottom latch boss",
        )

    rest_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: 0.70, latch_pivot: 0.0}):
        opened_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "flap swings forward from the upper hinge",
        rest_flap_aabb is not None
        and opened_flap_aabb is not None
        and opened_flap_aabb[1][1] > rest_flap_aabb[1][1] + 0.10
        and opened_flap_aabb[0][2] > rest_flap_aabb[0][2] + 0.10,
        details=f"rest={rest_flap_aabb}, opened={opened_flap_aabb}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(latch_tab, elem="tab_plate")
    with ctx.pose({flap_hinge: 0.0, latch_pivot: -0.60}):
        swung_tab_aabb = ctx.part_element_world_aabb(latch_tab, elem="tab_plate")
    ctx.check(
        "latch tab rotates on its own lower pivot",
        rest_tab_aabb is not None
        and swung_tab_aabb is not None
        and swung_tab_aabb[0][1] < rest_tab_aabb[0][1] - 0.010
        and swung_tab_aabb[0][2] > rest_tab_aabb[0][2] + 0.002,
        details=f"rest={rest_tab_aabb}, swung={swung_tab_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

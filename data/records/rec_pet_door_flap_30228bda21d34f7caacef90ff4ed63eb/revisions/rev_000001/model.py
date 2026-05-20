from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_CENTER_Z = 0.260
FRAME_OUTER = (0.360, 0.500)
FRAME_OPENING = (0.260, 0.320)
FRAME_DEPTH = 0.065
HINGE_Z = FRAME_CENTER_Z + FRAME_OPENING[1] / 2.0 - 0.008
HINGE_Y = -0.045


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_insert")

    white_plastic = Material("slightly_warm_white_plastic", (0.88, 0.86, 0.80, 1.0))
    gray_plastic = Material("dark_gray_slider_plastic", (0.12, 0.13, 0.14, 1.0))
    hinge_steel = Material("brushed_hinge_pin", (0.62, 0.62, 0.58, 1.0))
    smoky_frost = Material("smoky_translucent_flap", (0.30, 0.38, 0.42, 0.46))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                FRAME_OPENING,
                FRAME_OUTER,
                FRAME_DEPTH,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.012,
                outer_corner_radius=0.025,
            ),
            "frame_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, FRAME_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white_plastic,
        name="frame_ring",
    )

    # Exposed upper hinge: two fixed knuckles and a continuous pin carried by
    # the frame, with the moving flap knuckle interleaved between them.
    for idx, x in enumerate((-0.096, 0.096)):
        frame.visual(
            Box((0.064, 0.006, 0.018)),
            origin=Origin(xyz=(x, -0.035, HINGE_Z + 0.008)),
            material=white_plastic,
            name=f"hinge_tab_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.0075, length=0.055),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=white_plastic,
            name=f"fixed_knuckle_{idx}",
        )
    frame.visual(
        Cylinder(radius=0.0032, length=0.248),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    def add_track(
        x_center: float,
        *,
        back: str,
        upper_lip: str,
        lower_lip: str,
        end_stop_0: str,
        end_stop_1: str,
    ) -> None:
        # A shallow U-channel mounted on the lower frame rail.  The back plate
        # reaches the frame surface; the upper/lower lips capture the slider.
        frame.visual(
            Box((0.100, 0.030, 0.030)),
            origin=Origin(xyz=(x_center, -0.0475, 0.071)),
            material=white_plastic,
            name=back,
        )
        frame.visual(
            Box((0.100, 0.012, 0.004)),
            origin=Origin(xyz=(x_center, -0.068, 0.086)),
            material=white_plastic,
            name=upper_lip,
        )
        frame.visual(
            Box((0.100, 0.012, 0.004)),
            origin=Origin(xyz=(x_center, -0.068, 0.056)),
            material=white_plastic,
            name=lower_lip,
        )
        frame.visual(
            Box((0.006, 0.014, 0.030)),
            origin=Origin(xyz=(x_center - 0.050, -0.068, 0.071)),
            material=white_plastic,
            name=end_stop_0,
        )
        frame.visual(
            Box((0.006, 0.014, 0.030)),
            origin=Origin(xyz=(x_center + 0.050, -0.068, 0.071)),
            material=white_plastic,
            name=end_stop_1,
        )

    add_track(
        -0.090,
        back="track_0_back",
        upper_lip="track_0_upper_lip",
        lower_lip="track_0_lower_lip",
        end_stop_0="track_0_end_stop_0",
        end_stop_1="track_0_end_stop_1",
    )
    add_track(
        0.090,
        back="track_1_back",
        upper_lip="track_1_upper_lip",
        lower_lip="track_1_lower_lip",
        end_stop_0="track_1_end_stop_0",
        end_stop_1="track_1_end_stop_1",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.0065, length=0.130),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoky_frost,
        name="flap_knuckle",
    )
    flap.visual(
        Box((0.236, 0.010, 0.294)),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=smoky_frost,
        name="flap_panel",
    )
    flap.visual(
        Box((0.210, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, -0.288)),
        material=gray_plastic,
        name="bottom_weight",
    )

    def make_slider(name: str) -> object:
        slider = model.part(name)
        slider.visual(
            Box((0.034, 0.012, 0.016)),
            material=gray_plastic,
            name="thumb",
        )
        for idx, x in enumerate((-0.009, 0.0, 0.009)):
            slider.visual(
                Box((0.0025, 0.004, 0.014)),
                origin=Origin(xyz=(x, -0.0075, 0.0)),
                material=white_plastic,
                name=f"grip_{idx}",
            )
        return slider

    slider_0 = make_slider("lock_slider_0")
    slider_1 = make_slider("lock_slider_1")

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "frame_to_slider_0",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider_0,
        origin=Origin(xyz=(-0.110, -0.0685, 0.071)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.12, lower=0.0, upper=0.035),
    )
    model.articulation(
        "frame_to_slider_1",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider_1,
        origin=Origin(xyz=(0.110, -0.0685, 0.071)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.12, lower=0.0, upper=0.035),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    slider_0 = object_model.get_part("lock_slider_0")
    slider_1 = object_model.get_part("lock_slider_1")
    flap_joint = object_model.get_articulation("frame_to_flap")
    slider_joint_0 = object_model.get_articulation("frame_to_slider_0")
    slider_joint_1 = object_model.get_articulation("frame_to_slider_1")

    ctx.allow_overlap(
        frame,
        flap,
        elem_a="hinge_pin",
        elem_b="flap_knuckle",
        reason="The metal hinge pin is intentionally captured inside the flap's hinge knuckle.",
    )
    ctx.expect_overlap(
        frame,
        flap,
        axes="x",
        elem_a="hinge_pin",
        elem_b="flap_knuckle",
        min_overlap=0.120,
        name="hinge pin spans moving knuckle",
    )
    ctx.expect_gap(
        frame,
        flap,
        axis="y",
        positive_elem="frame_ring",
        negative_elem="flap_panel",
        min_gap=0.004,
        max_gap=0.020,
        name="closed flap sits just proud of frame",
    )

    for slider, track_name in ((slider_0, "track_0_back"), (slider_1, "track_1_back")):
        ctx.expect_within(
            slider,
            frame,
            axes="xz",
            inner_elem="thumb",
            outer_elem=track_name,
            margin=0.001,
            name=f"{slider.name} retained in side track",
        )
        ctx.expect_gap(
            frame,
            slider,
            axis="y",
            positive_elem=track_name,
            negative_elem="thumb",
            max_gap=0.001,
            max_penetration=0.0002,
            name=f"{slider.name} bears against track back plate",
        )

    rest_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 0.70}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    rest_flap_y = None if rest_flap_aabb is None else (rest_flap_aabb[0][1] + rest_flap_aabb[1][1]) / 2.0
    open_flap_y = None if open_flap_aabb is None else (open_flap_aabb[0][1] + open_flap_aabb[1][1]) / 2.0
    ctx.check(
        "flap swings outward about upper horizontal hinge",
        rest_flap_y is not None and open_flap_y is not None and open_flap_y > rest_flap_y + 0.050,
        details=f"rest_y={rest_flap_y}, open_y={open_flap_y}",
    )

    rest_0 = ctx.part_world_position(slider_0)
    rest_1 = ctx.part_world_position(slider_1)
    with ctx.pose({slider_joint_0: 0.035, slider_joint_1: 0.035}):
        moved_0 = ctx.part_world_position(slider_0)
        moved_1 = ctx.part_world_position(slider_1)
        ctx.expect_within(
            slider_0,
            frame,
            axes="xz",
            inner_elem="thumb",
            outer_elem="track_0_back",
            margin=0.001,
            name="lock_slider_0 retained when engaged",
        )
        ctx.expect_within(
            slider_1,
            frame,
            axes="xz",
            inner_elem="thumb",
            outer_elem="track_1_back",
            margin=0.001,
            name="lock_slider_1 retained when engaged",
        )
    ctx.check(
        "lock sliders move inward in side channels",
        rest_0 is not None
        and rest_1 is not None
        and moved_0 is not None
        and moved_1 is not None
        and moved_0[0] > rest_0[0] + 0.030
        and moved_1[0] < rest_1[0] - 0.030,
        details=f"rest_0={rest_0}, moved_0={moved_0}, rest_1={rest_1}, moved_1={moved_1}",
    )

    return ctx.report()


object_model = build_object_model()

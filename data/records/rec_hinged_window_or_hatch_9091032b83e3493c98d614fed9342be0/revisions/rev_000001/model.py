from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _ring_mesh(
    *,
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    height: float,
    z0: float,
    name: str,
):
    """Build a rounded rectangular ring as a managed mesh."""
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_size[0], outer_size[1], outer_radius, corner_segments=8),
        [rounded_rect_profile(inner_size[0], inner_size[1], inner_radius, corner_segments=8)],
        height,
        center=True,
    )
    geom.translate(0.0, 0.0, z0 + height * 0.5)
    return mesh_from_geometry(geom, name)


def _profile_at_z(
    width: float,
    depth: float,
    z: float,
    *,
    x_center: float,
    exponent: float = 3.2,
    segments: int = 72,
) -> list[tuple[float, float, float]]:
    return [(x_center + x, y, z) for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments)]


def _domed_lid_mesh(name: str):
    """A shallow, cambered hatch lid with a broad rounded-rectangle footprint."""
    x_center = 0.43
    sections = [
        _profile_at_z(0.860, 0.620, -0.020, x_center=x_center),
        _profile_at_z(0.860, 0.620, -0.006, x_center=x_center),
        _profile_at_z(0.845, 0.605, 0.004, x_center=x_center),
        _profile_at_z(0.810, 0.575, 0.017, x_center=x_center),
        _profile_at_z(0.740, 0.515, 0.030, x_center=x_center),
        _profile_at_z(0.520, 0.340, 0.039, x_center=x_center),
        _profile_at_z(0.120, 0.070, 0.042, x_center=x_center),
    ]
    return mesh_from_geometry(LoftGeometry(sections, cap=True, closed=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_deck_hatch")

    white_gelcoat = model.material("white_gelcoat", rgba=(0.86, 0.88, 0.84, 1.0))
    off_white = model.material("off_white", rgba=(0.78, 0.80, 0.76, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.025, 0.022, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_shadow = model.material("dark_recess", rgba=(0.06, 0.07, 0.07, 1.0))

    frame = model.part("curb_frame")
    frame.visual(
        _ring_mesh(
            outer_size=(1.02, 0.76),
            inner_size=(0.70, 0.46),
            outer_radius=0.070,
            inner_radius=0.040,
            height=0.026,
            z0=0.000,
            name="deck_flange",
        ),
        material=off_white,
        name="deck_flange",
    )
    frame.visual(
        _ring_mesh(
            outer_size=(0.80, 0.56),
            inner_size=(0.62, 0.38),
            outer_radius=0.055,
            inner_radius=0.035,
            height=0.102,
            z0=0.026,
            name="raised_curb",
        ),
        material=white_gelcoat,
        name="raised_curb",
    )
    frame.visual(
        _ring_mesh(
            outer_size=(0.69, 0.45),
            inner_size=(0.59, 0.35),
            outer_radius=0.040,
            inner_radius=0.028,
            height=0.008,
            z0=0.128,
            name="compression_gasket",
        ),
        material=black_rubber,
        name="compression_gasket",
    )
    frame.visual(Box((0.104, 0.115, 0.014)), origin=Origin(xyz=(-0.405, -0.245, 0.134)), material=stainless, name="hinge_leaf_0")
    frame.visual(Box((0.104, 0.115, 0.014)), origin=Origin(xyz=(-0.405, 0.245, 0.134)), material=stainless, name="hinge_leaf_1")
    frame.visual(Box((0.018, 0.105, 0.034)), origin=Origin(xyz=(-0.465, -0.245, 0.144)), material=stainless, name="hinge_web_0")
    frame.visual(Box((0.018, 0.105, 0.034)), origin=Origin(xyz=(-0.465, 0.245, 0.144)), material=stainless, name="hinge_web_1")
    frame.visual(Cylinder(radius=0.017, length=0.115), origin=Origin(xyz=(-0.452, -0.245, 0.164), rpy=(pi / 2.0, 0.0, 0.0)), material=stainless, name="hinge_barrel_0")
    frame.visual(Cylinder(radius=0.017, length=0.115), origin=Origin(xyz=(-0.452, 0.245, 0.164), rpy=(pi / 2.0, 0.0, 0.0)), material=stainless, name="hinge_barrel_1")
    frame.visual(Box((0.080, 0.070, 0.018)), origin=Origin(xyz=(0.405, -0.255, 0.113)), material=stainless, name="front_keeper_0")
    frame.visual(Box((0.080, 0.070, 0.018)), origin=Origin(xyz=(0.405, 0.255, 0.113)), material=stainless, name="front_keeper_1")
    frame.visual(Box((0.54, 0.30, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.029)), material=dark_shadow, name="hatch_opening_shadow")

    lid = model.part("lid")
    lid.visual(_domed_lid_mesh("domed_lid"), material=white_gelcoat, name="lid_shell")
    lid.visual(Box((0.105, 0.335, 0.011)), origin=Origin(xyz=(0.052, 0.0, 0.003)), material=stainless, name="lid_hinge_leaf")
    lid.visual(Cylinder(radius=0.0155, length=0.295), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=stainless, name="lid_hinge_barrel")
    for index, y in enumerate((-0.255, 0.255)):
        lid.visual(Cylinder(radius=0.030, length=0.024), origin=Origin(xyz=(0.790, y, 0.012)), material=white_gelcoat, name=f"latch_boss_{index}")

    dog_positions = (-0.255, 0.255)
    for index, y in enumerate(dog_positions):
        dog = model.part(f"latch_dog_{index}")
        dog.visual(Cylinder(radius=0.009, length=0.070), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=stainless, name="pivot_pin")
        dog.visual(Cylinder(radius=0.026, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.043)), material=stainless, name="top_washer")
        dog.visual(Box((0.125, 0.040, 0.014)), origin=Origin(xyz=(0.065, 0.0, -0.022)), material=stainless, name="dog_bar")
        dog.visual(Box((0.042, 0.050, 0.018)), origin=Origin(xyz=(0.085, 0.0, -0.028)), material=stainless, name="clamp_toe")
        dog.visual(Box((0.070, 0.030, 0.014)), origin=Origin(xyz=(-0.030, 0.0, 0.045)), material=stainless, name="turning_handle")
        model.articulation(
            f"lid_to_latch_dog_{index}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=dog,
            origin=Origin(xyz=(0.790, y, -0.005)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-1.35, upper=1.35),
        )

    model.articulation(
        "frame_to_lid",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(-0.452, 0.0, 0.164)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("curb_frame")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("frame_to_lid")
    dog_0 = object_model.get_part("latch_dog_0")
    dog_1 = object_model.get_part("latch_dog_1")
    dog_joint_0 = object_model.get_articulation("lid_to_latch_dog_0")
    dog_joint_1 = object_model.get_articulation("lid_to_latch_dog_1")

    for index, dog in enumerate((dog_0, dog_1)):
        ctx.allow_overlap(
            lid,
            dog,
            elem_a=f"latch_boss_{index}",
            elem_b="pivot_pin",
            reason="The short latch-dog pin is intentionally captured in the molded boss on the lid.",
        )
        ctx.allow_overlap(
            lid,
            dog,
            elem_a="lid_shell",
            elem_b="pivot_pin",
            reason="The latch-dog pivot pin intentionally passes through the lid shell at the front edge.",
        )
        ctx.expect_within(
            dog,
            lid,
            axes="xy",
            inner_elem="pivot_pin",
            outer_elem=f"latch_boss_{index}",
            margin=0.004,
            name=f"latch dog {index} pin is centered in its boss",
        )
        ctx.expect_overlap(
            dog,
            lid,
            axes="z",
            elem_a="pivot_pin",
            elem_b=f"latch_boss_{index}",
            min_overlap=0.010,
            name=f"latch dog {index} pin is captured vertically",
        )
        ctx.expect_overlap(
            dog,
            lid,
            axes="z",
            elem_a="pivot_pin",
            elem_b="lid_shell",
            min_overlap=0.015,
            name=f"latch dog {index} pin passes through the lid shell",
        )

    with ctx.pose({hinge: 0.0, dog_joint_0: 0.0, dog_joint_1: 0.0}):
        ctx.expect_overlap(
            lid,
            frame,
            axes="xy",
            elem_a="lid_shell",
            elem_b="raised_curb",
            min_overlap=0.50,
            name="closed lid footprint covers the raised curb",
        )
        ctx.expect_gap(
            lid,
            frame,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="compression_gasket",
            min_gap=0.002,
            max_gap=0.018,
            name="closed lid is seated just above the gasket",
        )
        ctx.expect_overlap(
            dog_0,
            frame,
            axes="xy",
            elem_a="clamp_toe",
            elem_b="front_keeper_0",
            min_overlap=0.015,
            name="first latch dog sits over its keeper",
        )
        ctx.expect_overlap(
            dog_1,
            frame,
            axes="xy",
            elem_a="clamp_toe",
            elem_b="front_keeper_1",
            min_overlap=0.015,
            name="second latch dog sits over its keeper",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge lifts the lid upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.22,
        details=f"closed={closed_aabb}, opened={open_aabb}",
    )

    closed_bar = ctx.part_element_world_aabb(dog_0, elem="dog_bar")
    with ctx.pose({dog_joint_0: 1.0}):
        swung_bar = ctx.part_element_world_aabb(dog_0, elem="dog_bar")
    if closed_bar is not None and swung_bar is not None:
        closed_center_y = 0.5 * (closed_bar[0][1] + closed_bar[1][1])
        swung_center_y = 0.5 * (swung_bar[0][1] + swung_bar[1][1])
        dog_swung = abs(swung_center_y - closed_center_y) > 0.04
    else:
        dog_swung = False
    ctx.check("latch dog rotates on its local pin", dog_swung, details=f"closed={closed_bar}, swung={swung_bar}")

    return ctx.report()


object_model = build_object_model()

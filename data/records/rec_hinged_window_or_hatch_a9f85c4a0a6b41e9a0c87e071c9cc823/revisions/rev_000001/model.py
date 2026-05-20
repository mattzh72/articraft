from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HINGE_X = -0.365
HINGE_Y = 0.055
PANEL_WIDTH = 0.535
PANEL_HEIGHT = 0.660
PANEL_CENTER_X = -HINGE_X
PANEL_CENTER_Y_LOCAL = -0.062
FREE_EDGE_X_LOCAL = PANEL_CENTER_X + PANEL_WIDTH / 2.0
TOP_LATCH_Z = 0.255
BOTTOM_LATCH_Z = -0.255


def _vertical_rounded_plate(width: float, height: float, radius: float, depth: float, name: str):
    """Rounded rectangle in the local XZ plane, with thickness along local Y."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        depth,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _vertical_frame_ring():
    """Thick aircraft-style jamb frame, open through the middle."""
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.700, 0.840, 0.055, corner_segments=12),
        [rounded_rect_profile(0.585, 0.730, 0.040, corner_segments=12)],
        0.045,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "outer_frame")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aircraft_access_hatch")

    frame_mat = model.material("painted_airframe_grey", rgba=(0.58, 0.62, 0.62, 1.0))
    panel_mat = model.material("recessed_panel_grey", rgba=(0.36, 0.39, 0.39, 1.0))
    raised_mat = model.material("raised_panel_rim", rgba=(0.50, 0.53, 0.53, 1.0))
    hinge_mat = model.material("dark_hinge_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    latch_mat = model.material("black_latch_dogs", rgba=(0.04, 0.045, 0.045, 1.0))
    keeper_mat = model.material("polished_wear_plate", rgba=(0.72, 0.70, 0.64, 1.0))

    frame = model.part("jamb_frame")
    frame.visual(
        Box((0.065, 0.045, 0.840)),
        origin=Origin(xyz=(-0.3175, 0.0, 0.0)),
        material=frame_mat,
        name="hinge_frame",
    )
    frame.visual(
        Box((0.065, 0.045, 0.840)),
        origin=Origin(xyz=(0.3175, 0.0, 0.0)),
        material=frame_mat,
        name="outer_frame",
    )
    frame.visual(
        Box((0.700, 0.045, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.3875)),
        material=frame_mat,
        name="top_frame",
    )
    frame.visual(
        Box((0.700, 0.045, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, -0.3875)),
        material=frame_mat,
        name="bottom_frame",
    )

    # Hinge-side jamb hardware: a raised standoff and three fixed barrels on the
    # pin line.  The panel barrels occupy the two clear gaps between these.
    frame.visual(
        Box((0.060, 0.018, 0.700)),
        origin=Origin(xyz=(-0.318, 0.029, 0.0)),
        material=hinge_mat,
        name="hinge_standoff",
    )
    fixed_barrels = (
        ("fixed_barrel_low", -0.2775, 0.125),
        ("fixed_barrel_mid", 0.0, 0.130),
        ("fixed_barrel_high", 0.2775, 0.125),
    )
    for name, zc, length in fixed_barrels:
        frame.visual(
            Box((0.032, 0.017, length)),
            origin=Origin(xyz=(-0.349, 0.046, zc)),
            material=hinge_mat,
            name=f"{name}_leaf",
        )
        frame.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(HINGE_X, HINGE_Y, zc)),
            material=hinge_mat,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.006, length=0.670),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        material=hinge_mat,
        name="hinge_pin",
    )

    # Wear plates on the free jamb where the rotary latch dogs bear.
    for name, zc in (("top_keeper", TOP_LATCH_Z), ("bottom_keeper", BOTTOM_LATCH_Z)):
        frame.visual(
            Box((0.070, 0.016, 0.070)),
            origin=Origin(xyz=(0.322, 0.0305, zc)),
            material=keeper_mat,
            name=name,
        )

    panel = model.part("access_panel")
    panel.visual(
        _vertical_rounded_plate(PANEL_WIDTH, PANEL_HEIGHT, 0.032, 0.022, "panel_skin"),
        origin=Origin(xyz=(PANEL_CENTER_X, PANEL_CENTER_Y_LOCAL, 0.0)),
        material=panel_mat,
        name="panel_skin",
    )
    # Raised border lip leaves the center field visibly recessed inside the frame.
    panel.visual(
        Box((PANEL_WIDTH - 0.020, 0.010, 0.034)),
        origin=Origin(xyz=(PANEL_CENTER_X, -0.046, PANEL_HEIGHT / 2.0 - 0.020)),
        material=raised_mat,
        name="top_panel_lip",
    )
    panel.visual(
        Box((PANEL_WIDTH - 0.020, 0.010, 0.034)),
        origin=Origin(xyz=(PANEL_CENTER_X, -0.046, -PANEL_HEIGHT / 2.0 + 0.020)),
        material=raised_mat,
        name="bottom_panel_lip",
    )
    panel.visual(
        Box((0.034, 0.010, PANEL_HEIGHT - 0.030)),
        origin=Origin(xyz=(PANEL_CENTER_X - PANEL_WIDTH / 2.0 + 0.020, -0.046, 0.0)),
        material=raised_mat,
        name="hinge_panel_lip",
    )
    panel.visual(
        Box((0.034, 0.010, PANEL_HEIGHT - 0.030)),
        origin=Origin(xyz=(PANEL_CENTER_X + PANEL_WIDTH / 2.0 - 0.020, -0.046, 0.0)),
        material=raised_mat,
        name="free_panel_lip",
    )

    # Panel-side hinge strap: a high bridge over the fixed frame, local leaf
    # tabs only at the moving barrel locations, and a downstand inside the
    # opening that ties the strap into the recessed panel skin.
    panel.visual(
        Box((0.075, 0.012, 0.610)),
        origin=Origin(xyz=(0.073, 0.010, 0.0)),
        material=hinge_mat,
        name="hinge_bridge",
    )
    panel.visual(
        Box((0.024, 0.068, 0.610)),
        origin=Origin(xyz=(0.112, -0.024, 0.0)),
        material=hinge_mat,
        name="hinge_downstand",
    )
    for name, zc in (("moving_barrel_low", -0.140), ("moving_barrel_high", 0.140)):
        panel.visual(
            Box((0.040, 0.018, 0.125)),
            origin=Origin(xyz=(0.029, 0.006, zc)),
            material=hinge_mat,
            name=f"{name}_leaf",
        )
        panel.visual(
            Cylinder(radius=0.014, length=0.125),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hinge_mat,
            name=name,
        )

    # Bosses under the two latch pivots; the separate latch dog links sit on
    # these circular pads and rotate around their own local Y axes.
    for name, zc in (("top_pivot_boss", TOP_LATCH_Z), ("bottom_pivot_boss", BOTTOM_LATCH_Z)):
        panel.visual(
            Cylinder(radius=0.028, length=0.008),
            origin=Origin(
                xyz=(FREE_EDGE_X_LOCAL - 0.030, -0.052, zc),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=raised_mat,
            name=f"{name}_flange",
        )
        panel.visual(
            Cylinder(radius=0.019, length=0.037),
            origin=Origin(
                xyz=(FREE_EDGE_X_LOCAL - 0.030, -0.0325, zc),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_mat,
            name=name,
        )

    def add_latch_dog(part_name: str) -> object:
        dog = model.part(part_name)
        dog.visual(
            Box((0.103, 0.014, 0.028)),
            origin=Origin(xyz=(0.050, 0.0, 0.0)),
            material=latch_mat,
            name="dog_bar",
        )
        dog.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=latch_mat,
            name="pivot_hub",
        )
        dog.visual(
            Cylinder(radius=0.016, length=0.016),
            origin=Origin(xyz=(0.103, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=latch_mat,
            name="nose_pad",
        )
        return dog

    top_dog = add_latch_dog("latch_dog_0")
    bottom_dog = add_latch_dog("latch_dog_1")

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    for joint_name, dog, zc in (
        ("latch_pivot_0", top_dog, TOP_LATCH_Z),
        ("latch_pivot_1", bottom_dog, BOTTOM_LATCH_Z),
    ):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=panel,
            child=dog,
            origin=Origin(xyz=(FREE_EDGE_X_LOCAL - 0.030, -0.005, zc)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.25, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("jamb_frame")
    panel = object_model.get_part("access_panel")
    top_dog = object_model.get_part("latch_dog_0")
    bottom_dog = object_model.get_part("latch_dog_1")
    hinge = object_model.get_articulation("panel_hinge")
    top_pivot = object_model.get_articulation("latch_pivot_0")

    ctx.allow_overlap(
        frame,
        panel,
        elem_a="hinge_pin",
        elem_b="moving_barrel_low",
        reason="The fixed hinge pin is intentionally captured inside the moving lower hinge barrel.",
    )
    ctx.allow_overlap(
        frame,
        panel,
        elem_a="hinge_pin",
        elem_b="moving_barrel_high",
        reason="The fixed hinge pin is intentionally captured inside the moving upper hinge barrel.",
    )
    ctx.expect_within(
        frame,
        panel,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="moving_barrel_low",
        margin=0.001,
        name="lower moving barrel is clipped around hinge pin",
    )
    ctx.expect_within(
        frame,
        panel,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="moving_barrel_high",
        margin=0.001,
        name="upper moving barrel is clipped around hinge pin",
    )
    ctx.expect_overlap(
        frame,
        panel,
        axes="z",
        elem_a="hinge_pin",
        elem_b="moving_barrel_low",
        min_overlap=0.100,
        name="lower barrel retains hinge pin length",
    )
    ctx.expect_overlap(
        frame,
        panel,
        axes="z",
        elem_a="hinge_pin",
        elem_b="moving_barrel_high",
        min_overlap=0.100,
        name="upper barrel retains hinge pin length",
    )

    frame_aabb = ctx.part_element_world_aabb(frame, elem="outer_frame")
    panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_skin")
    ctx.check(
        "panel is recessed behind the thick frame face",
        frame_aabb is not None
        and panel_aabb is not None
        and frame_aabb[1][1] > panel_aabb[1][1] + 0.012,
        details=f"frame={frame_aabb}, panel={panel_aabb}",
    )

    # The closed latch dogs sit just over the free-edge wear plates rather than
    # floating far away from the jamb.
    ctx.expect_gap(
        top_dog,
        frame,
        axis="y",
        positive_elem="nose_pad",
        negative_elem="top_keeper",
        min_gap=0.0,
        max_gap=0.006,
        name="top latch dog bears over top keeper",
    )
    ctx.expect_gap(
        bottom_dog,
        frame,
        axis="y",
        positive_elem="nose_pad",
        negative_elem="bottom_keeper",
        min_gap=0.0,
        max_gap=0.006,
        name="bottom latch dog bears over bottom keeper",
    )
    ctx.expect_overlap(
        top_dog,
        frame,
        axes="xz",
        elem_a="nose_pad",
        elem_b="top_keeper",
        min_overlap=0.015,
        name="top dog overlaps its keeper footprint",
    )
    ctx.expect_overlap(
        bottom_dog,
        frame,
        axes="xz",
        elem_a="nose_pad",
        elem_b="bottom_keeper",
        min_overlap=0.015,
        name="bottom dog overlaps its keeper footprint",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_skin")
    with ctx.pose({hinge: 1.20}):
        open_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_skin")
    ctx.check(
        "panel swings outward on vertical side hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.18,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    closed_nose = ctx.part_element_world_aabb(top_dog, elem="nose_pad")
    with ctx.pose({top_pivot: 1.00}):
        turned_nose = ctx.part_element_world_aabb(top_dog, elem="nose_pad")
    ctx.check(
        "latch dog rotates on its own pivot",
        closed_nose is not None
        and turned_nose is not None
        and abs(((closed_nose[0][2] + closed_nose[1][2]) / 2.0) - ((turned_nose[0][2] + turned_nose[1][2]) / 2.0)) > 0.040,
        details=f"closed={closed_nose}, turned={turned_nose}",
    )

    return ctx.report()


object_model = build_object_model()

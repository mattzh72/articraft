from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HOUSING_W = 0.54
HOUSING_D = 0.18
HOUSING_H = 0.82

DOOR_W = 0.46
DOOR_T = 0.026
DOOR_H = 0.57
HINGE_Y = 0.205
HINGE_Z = 0.125


def _safe_front_fillet(model: cq.Workplane, radius: float) -> cq.Workplane:
    """Add a soft medical-appliance edge treatment without making construction brittle."""
    try:
        return model.faces(">Y").edges().fillet(radius)
    except Exception:
        return model


def _housing_shell() -> cq.Workplane:
    """Single connected, flat-backed purifier shell with a real recessed filter opening."""
    body = cq.Workplane("XY").box(HOUSING_W, HOUSING_D, HOUSING_H).translate(
        (0.0, HOUSING_D / 2.0, HOUSING_H / 2.0)
    )
    body = _safe_front_fillet(body, 0.014)

    # Cut only from the front toward the back so the rear wall stays flat and closed.
    cutter = cq.Workplane("XY").box(0.430, 0.300, 0.565).translate((0.0, 0.180, 0.410))
    body = body.cut(cutter)

    # A small lower service-lip relief makes the hinge area read as a service door sill.
    lip_cutter = cq.Workplane("XY").box(0.500, 0.090, 0.026).translate((0.0, 0.190, 0.112))
    body = body.cut(lip_cutter)
    return body


def _door_panel() -> cq.Workplane:
    """Rounded but flat rectangular access door, authored in the child hinge frame."""
    # The lower edge clears the hinge pin; the hinge leaf below visibly carries the load.
    panel = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H).translate((0.0, 0.0, DOOR_H / 2.0 + 0.014))
    panel = _safe_front_fillet(panel, 0.010)
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_medical_air_purifier")

    shell_mat = model.material("antimicrobial_warm_white", rgba=(0.86, 0.88, 0.88, 1.0))
    door_mat = model.material("powder_coated_white", rgba=(0.93, 0.94, 0.93, 1.0))
    trim_mat = model.material("soft_gray_trim", rgba=(0.62, 0.66, 0.68, 1.0))
    shadow_mat = model.material("cavity_shadow", rgba=(0.025, 0.030, 0.034, 1.0))
    filter_mat = model.material("hepa_filter_media", rgba=(0.72, 0.78, 0.80, 1.0))
    gasket_mat = model.material("blue_medical_gasket", rgba=(0.18, 0.46, 0.78, 1.0))
    metal_mat = model.material("brushed_stainless", rgba=(0.65, 0.67, 0.66, 1.0))
    latch_mat = model.material("satin_latch_gray", rgba=(0.40, 0.43, 0.44, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "housing_shell", tolerance=0.0008),
        material=shell_mat,
        name="housing_shell",
    )

    # Recessed filter cavity with visible pleated HEPA media behind the access panel.
    housing.visual(
        Box((0.402, 0.012, 0.498)),
        origin=Origin(xyz=(0.0, 0.034, 0.410)),
        material=shadow_mat,
        name="cavity_shadow",
    )
    housing.visual(
        Box((0.382, 0.010, 0.476)),
        origin=Origin(xyz=(0.0, 0.042, 0.410)),
        material=filter_mat,
        name="filter_media",
    )
    for i, x in enumerate([-0.165, -0.132, -0.099, -0.066, -0.033, 0.0, 0.033, 0.066, 0.099, 0.132, 0.165]):
        housing.visual(
            Box((0.010, 0.012, 0.462)),
            origin=Origin(xyz=(x, 0.050, 0.410)),
            material=model.material(f"pleat_shade_{i}", rgba=(0.62, 0.70, 0.73, 1.0))
            if i % 2
            else filter_mat,
            name=f"filter_pleat_{i}",
        )

    cavity_bezel = BezelGeometry(
        (0.410, 0.535),
        (0.468, 0.592),
        0.007,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.010,
        outer_corner_radius=0.018,
    )
    housing.visual(
        mesh_from_geometry(cavity_bezel, "cavity_bezel"),
        origin=Origin(xyz=(0.0, 0.183, 0.410), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket_mat,
        name="cavity_bezel",
    )

    # Fixed half of a continuous knuckle hinge and its load path back into the housing.
    housing.visual(
        Box((0.500, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, 0.188, 0.077)),
        material=shell_mat,
        name="hinge_support",
    )
    housing.visual(
        Box((0.500, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.195, 0.105)),
        material=metal_mat,
        name="hinge_leaf",
    )

    hinge_centers = [-0.216, -0.162, -0.108, -0.054, 0.0, 0.054, 0.108, 0.162, 0.216]
    for i, x in enumerate(hinge_centers):
        if i % 2 == 0:
            housing.visual(
                Cylinder(radius=0.008, length=0.040),
                origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal_mat,
                name=f"fixed_knuckle_{i}",
            )

    # Upper corner detent keepers attached to the upper housing frame.
    for i, x in enumerate((-0.188, 0.188)):
        keeper_arm_name = ("keeper_arm_0", "keeper_arm_1")[i]
        keeper_name = ("keeper_0", "keeper_1")[i]
        housing.visual(
            Box((0.040, 0.044, 0.010)),
            origin=Origin(xyz=(x, 0.202, 0.720)),
            material=metal_mat,
            name=keeper_arm_name,
        )
        housing.visual(
            Box((0.044, 0.008, 0.024)),
            origin=Origin(xyz=(x, 0.224, 0.729)),
            material=metal_mat,
            name=keeper_name,
        )

    door = model.part("filter_door")
    door.visual(
        mesh_from_cadquery(_door_panel(), "door_panel", tolerance=0.0008),
        material=door_mat,
        name="door_panel",
    )
    # A raised perimeter lip on the front panel reads as a gasketed medical service door.
    door.visual(
        Box((0.430, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, 0.017, 0.548)),
        material=trim_mat,
        name="top_lip",
    )
    door.visual(
        Box((0.430, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.017, 0.030)),
        material=trim_mat,
        name="bottom_lip",
    )
    for i, x in enumerate((-0.219, 0.219)):
        door.visual(
            Box((0.020, 0.008, 0.520)),
            origin=Origin(xyz=(x, 0.017, 0.286)),
            material=trim_mat,
            name=f"side_lip_{i}",
        )

    grille = VentGrilleGeometry(
        (0.358, 0.086),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=0.012,
        duct_wall=0.0025,
        slat_pitch=0.013,
        slat_width=0.006,
        slat_angle_deg=32.0,
        corner_radius=0.006,
        slats=VentGrilleSlats(profile="airfoil", direction="down", inset=0.002, divider_count=2),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
        sleeve=VentGrilleSleeve(style="short", depth=0.010, wall=0.0025),
    )
    door.visual(
        mesh_from_geometry(grille, "front_grille"),
        origin=Origin(xyz=(0.0, 0.015, 0.392), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="front_grille",
    )

    # Moving half of the piano hinge: alternating knuckles plus a leaf tied to the door.
    door.visual(
        Box((0.460, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, 0.012, 0.016)),
        material=metal_mat,
        name="door_hinge_leaf",
    )
    for i, x in enumerate(hinge_centers):
        if i % 2 == 1:
            door.visual(
                Cylinder(radius=0.008, length=0.040),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal_mat,
                name=f"door_knuckle_{i}",
            )

    # Stationary bosses on the door carry the two rotating upper-corner latches.
    latch_origins = [(-0.188, 0.019, 0.520), (0.188, 0.019, 0.520)]
    for i, (x, y, z) in enumerate(latch_origins):
        latch_boss_name = ("latch_boss_0", "latch_boss_1")[i]
        door.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(x, y - 0.003, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name=latch_boss_name,
        )

    model.articulation(
        "housing_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    for i, (x, y, z) in enumerate(latch_origins):
        latch = model.part(f"upper_latch_{i}")
        latch.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=latch_mat,
            name="latch_disk",
        )
        latch.visual(
            Box((0.030, 0.006, 0.090)),
            origin=Origin(xyz=(0.0, 0.008, 0.045)),
            material=latch_mat,
            name="latch_tab",
        )
        latch.visual(
            Box((0.038, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, 0.010, 0.086)),
            material=latch_mat,
            name="detent_hook",
        )
        model.articulation(
            f"door_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=door,
            child=latch,
            origin=Origin(xyz=(x, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.65, upper=0.65),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("filter_door")
    hinge = object_model.get_articulation("housing_to_filter_door")
    latch_0 = object_model.get_part("upper_latch_0")
    latch_1 = object_model.get_part("upper_latch_1")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            positive_elem="door_panel",
            negative_elem="cavity_bezel",
            min_gap=0.002,
            max_gap=0.012,
            name="closed door seats just proud of the filter-cavity bezel",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="xz",
            elem_a="door_panel",
            elem_b="cavity_bezel",
            min_overlap=0.40,
            name="door covers the recessed filter cavity",
        )
        ctx.expect_contact(
            latch_0,
            door,
            elem_a="latch_disk",
            elem_b="latch_boss_0",
            contact_tol=0.0015,
            name="first detent latch is carried on a door boss",
        )
        ctx.expect_contact(
            latch_1,
            door,
            elem_a="latch_disk",
            elem_b="latch_boss_1",
            contact_tol=0.0015,
            name="second detent latch is carried on a door boss",
        )
        ctx.expect_gap(
            latch_0,
            housing,
            axis="y",
            positive_elem="detent_hook",
            negative_elem="keeper_0",
            min_gap=0.0005,
            max_gap=0.006,
            name="first detent hook sits in front of its keeper",
        )
        ctx.expect_gap(
            latch_1,
            housing,
            axis="y",
            positive_elem="detent_hook",
            negative_elem="keeper_1",
            min_gap=0.0005,
            max_gap=0.006,
            name="second detent hook sits in front of its keeper",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: 1.70}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "filter door folds downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.30
        and open_aabb[1][2] < closed_aabb[1][2] - 0.18,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    ctx.check(
        "base hinge has realistic downward service-door travel",
        hinge.motion_limits is not None and hinge.motion_limits.lower == 0.0 and hinge.motion_limits.upper >= 1.6,
        details=f"limits={hinge.motion_limits}",
    )
    return ctx.report()


object_model = build_object_model()

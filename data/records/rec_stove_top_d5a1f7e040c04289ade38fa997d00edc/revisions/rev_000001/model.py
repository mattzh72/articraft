from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


SLAB_W = 1.05
SLAB_D = 0.72
SLAB_T = 0.040
GLASS_W = 0.780
GLASS_D = 0.560
GLASS_T = 0.012
GLASS_TOP_Z = SLAB_T + 0.003
GLASS_CENTER_Z = GLASS_TOP_Z - GLASS_T / 2.0
CUTOUT_W = 0.812
CUTOUT_D = 0.592
BUTTON_TRAVEL = 0.006
BUTTON_HOLE_R = 0.0180
BUTTON_CAP_D = 0.0340

BUTTON_CENTERS = (
    (-0.325, -0.235),
    (-0.275, -0.235),
    (-0.325, -0.185),
    (-0.275, -0.185),
)


def _through_cylinder(x: float, y: float, radius: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(depth)
        .translate((0.0, 0.0, -depth / 2.0))
    )


def _countertop_slab() -> cq.Workplane:
    slab = cq.Workplane("XY").box(SLAB_W, SLAB_D, SLAB_T)
    cutter = (
        cq.Workplane("XY")
        .rect(CUTOUT_W, CUTOUT_D)
        .extrude(SLAB_T * 3.0)
        .translate((0.0, 0.0, -SLAB_T * 1.5))
    )
    return slab.cut(cutter).edges("|Z").fillet(0.010)


def _glass_panel() -> cq.Workplane:
    glass = cq.Workplane("XY").box(GLASS_W, GLASS_D, GLASS_T).edges("|Z").fillet(0.024)
    for x, y in BUTTON_CENTERS:
        glass = glass.cut(_through_cylinder(x, y, BUTTON_HOLE_R, GLASS_T * 3.0))
    return glass


def _annular_ring(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_ceramic_cooktop")

    stone = model.material("warm_stone", color=(0.70, 0.66, 0.58, 1.0))
    black_glass = model.material("black_ceramic_glass", color=(0.010, 0.012, 0.014, 1.0))
    gasket = model.material("matte_black_gasket", color=(0.004, 0.004, 0.004, 1.0))
    zone_print = model.material("pale_zone_graphics", color=(0.78, 0.80, 0.78, 0.92))
    metal = model.material("satin_button_metal", color=(0.72, 0.70, 0.66, 1.0))
    dark_plastic = model.material("black_button_stem", color=(0.025, 0.025, 0.025, 1.0))

    countertop = model.part("countertop")
    countertop.visual(
        mesh_from_cadquery(_countertop_slab(), "countertop_slab", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, SLAB_T / 2.0)),
        material=stone,
        name="stone_slab",
    )
    countertop.visual(
        mesh_from_cadquery(_glass_panel(), "glass_panel", tolerance=0.0006),
        origin=Origin(xyz=(0.0, 0.0, GLASS_CENTER_Z)),
        material=black_glass,
        name="glass_panel",
    )

    # The thin gasket frame visibly mounts the glass into the countertop cutout
    # while keeping the object a countertop cooktop, not a stove range.
    frame_t = 0.004
    frame_w = 0.032
    frame_z = SLAB_T + frame_t / 2.0 - 0.0005
    countertop.visual(
        Box((CUTOUT_W + 0.024, frame_w, frame_t)),
        origin=Origin(xyz=(0.0, -CUTOUT_D / 2.0 + 0.006, frame_z)),
        material=gasket,
        name="front_gasket",
    )
    countertop.visual(
        Box((CUTOUT_W + 0.024, frame_w, frame_t)),
        origin=Origin(xyz=(0.0, CUTOUT_D / 2.0 - 0.006, frame_z)),
        material=gasket,
        name="rear_gasket",
    )
    countertop.visual(
        Box((frame_w, CUTOUT_D - 0.012, frame_t)),
        origin=Origin(xyz=(-CUTOUT_W / 2.0 + 0.006, 0.0, frame_z)),
        material=gasket,
        name="side_gasket_0",
    )
    countertop.visual(
        Box((frame_w, CUTOUT_D - 0.012, frame_t)),
        origin=Origin(xyz=(CUTOUT_W / 2.0 - 0.006, 0.0, frame_z)),
        material=gasket,
        name="side_gasket_1",
    )

    # Printed ceramic cooking zones: four flat rings on the glass surface.
    zones = (
        ("zone_front_0", -0.175, -0.070, 0.076, 0.071),
        ("zone_rear_0", -0.185, 0.165, 0.091, 0.085),
        ("zone_front_1", 0.205, -0.090, 0.100, 0.093),
        ("zone_rear_1", 0.215, 0.155, 0.078, 0.073),
    )
    for name, x, y, outer_r, inner_r in zones:
        countertop.visual(
            mesh_from_cadquery(_annular_ring(outer_r, inner_r, 0.0008), name, tolerance=0.0004),
            origin=Origin(xyz=(x, y, GLASS_TOP_Z + 0.00015)),
            material=zone_print,
            name=name,
        )
    countertop.visual(
        mesh_from_cadquery(_annular_ring(0.071, 0.066, 0.0008), "zone_front_1_inner", tolerance=0.0004),
        origin=Origin(xyz=(0.205, -0.090, GLASS_TOP_Z + 0.00018)),
        material=zone_print,
        name="zone_front_1_inner",
    )
    countertop.visual(
        mesh_from_cadquery(_annular_ring(0.062, 0.058, 0.0008), "zone_rear_0_inner", tolerance=0.0004),
        origin=Origin(xyz=(-0.185, 0.165, GLASS_TOP_Z + 0.00018)),
        material=zone_print,
        name="zone_rear_0_inner",
    )

    # Four compact, physical push-button collars in the left-front glass corner.
    collar_mesh = mesh_from_cadquery(_annular_ring(0.023, 0.0185, 0.0012), "button_collar", tolerance=0.0004)
    for idx, (x, y) in enumerate(BUTTON_CENTERS):
        row = idx // 2
        col = idx % 2
        countertop.visual(
            collar_mesh,
            origin=Origin(xyz=(x, y, GLASS_TOP_Z - 0.00060)),
            material=gasket,
            name=f"button_collar_{row}_{col}",
        )

    button_cap_mesh = mesh_from_geometry(
        KnobGeometry(
            BUTTON_CAP_D,
            0.008,
            body_style="domed",
            crown_radius=0.0025,
            edge_radius=0.0010,
            center=False,
        ),
        "button_cap",
    )

    for idx, (x, y) in enumerate(BUTTON_CENTERS):
        row = idx // 2
        col = idx % 2
        button = model.part(f"button_{row}_{col}")
        button.visual(
            button_cap_mesh,
            origin=Origin(),
            material=metal,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=dark_plastic,
            name="stem",
        )
        button.visual(
            Cylinder(radius=0.024, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, -GLASS_T - 0.001)),
            material=dark_plastic,
            name="retaining_washer",
        )
        model.articulation(
            f"countertop_to_button_{row}_{col}",
            ArticulationType.PRISMATIC,
            parent=countertop,
            child=button,
            origin=Origin(xyz=(x, y, GLASS_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    countertop = object_model.get_part("countertop")
    joints = tuple(object_model.articulations)

    ctx.check(
        "only four controls articulate",
        len(joints) == 4 and all(j.articulation_type == ArticulationType.PRISMATIC for j in joints),
        details=f"articulations={[j.name for j in joints]}",
    )

    button_names = {f"button_{r}_{c}" for r in range(2) for c in range(2)}
    child_names = {j.child if isinstance(j.child, str) else j.child.name for j in joints}
    ctx.check(
        "only buttons are child links",
        child_names == button_names,
        details=f"children={sorted(child_names)}",
    )
    ctx.check(
        "four cooking zones are marked on the glass",
        all(object_model.get_part("countertop").get_visual(name) is not None for name in (
            "zone_front_0",
            "zone_rear_0",
            "zone_front_1",
            "zone_rear_1",
        )),
        details="expected four primary circular zone graphics",
    )

    rest_positions = {
        f"button_{idx // 2}_{idx % 2}": (x, y)
        for idx, (x, y) in enumerate(BUTTON_CENTERS)
    }
    xs = [xy[0] for xy in rest_positions.values()]
    ys = [xy[1] for xy in rest_positions.values()]
    ctx.check(
        "button group sits at the left front corner",
        max(xs) < -0.25 and max(ys) < -0.17 and (max(xs) - min(xs)) <= 0.055 and (max(ys) - min(ys)) <= 0.055,
        details=f"button_xy={rest_positions}",
    )

    for row in range(2):
        for col in range(2):
            button = object_model.get_part(f"button_{row}_{col}")
            joint = object_model.get_articulation(f"countertop_to_button_{row}_{col}")
            ctx.check(
                f"button_{row}_{col} plunger axis is panel-normal",
                tuple(joint.axis) == (0.0, 0.0, -1.0),
                details=f"axis={joint.axis}",
            )
            ctx.expect_contact(
                button,
                countertop,
                elem_a="retaining_washer",
                elem_b="glass_panel",
                contact_tol=0.00001,
                name=f"button_{row}_{col} is retained under the glass",
            )
            ctx.expect_gap(
                button,
                countertop,
                axis="z",
                positive_elem="cap",
                negative_elem="glass_panel",
                max_gap=0.001,
                max_penetration=0.0002,
                name=f"button_{row}_{col} rests at the glass face",
            )
            rest = ctx.part_world_position(button)
            with ctx.pose({joint: BUTTON_TRAVEL}):
                pressed = ctx.part_world_position(button)
                ctx.expect_gap(
                    button,
                    countertop,
                    axis="z",
                    positive_elem="cap",
                    negative_elem="glass_panel",
                    max_penetration=BUTTON_TRAVEL + 0.001,
                    name=f"button_{row}_{col} plunges normal to panel",
                )
            ctx.check(
                f"button_{row}_{col} moves downward",
                rest is not None
                and pressed is not None
                and pressed[2] < rest[2] - BUTTON_TRAVEL * 0.8,
                details=f"rest={rest}, pressed={pressed}",
            )

    return ctx.report()


object_model = build_object_model()

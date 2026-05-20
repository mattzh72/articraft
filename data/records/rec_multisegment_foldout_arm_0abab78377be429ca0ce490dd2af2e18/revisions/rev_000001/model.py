from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PLATE_HEIGHT = 0.052
PLATE_THICKNESS = 0.026
HINGE_RADIUS = 0.018
HINGE_LENGTH = 0.050
ARM_Z = 0.120


def _circle_profile(cx: float, cy: float, radius: float, samples: int = 28):
    return [
        (cx + radius * cos(tau * i / samples), cy + radius * sin(tau * i / samples))
        for i in range(samples)
    ]


def _slot_profile(cx: float, cy: float, width: float, height: float, samples: int = 10):
    """A small rounded inspection/lightening slot for the side plates."""
    radius = height * 0.5
    straight = max(0.0, width * 0.5 - radius)
    pts = []
    for i in range(samples + 1):
        a = -pi / 2.0 + pi * i / samples
        pts.append((cx + straight + radius * cos(a), cy + radius * sin(a)))
    for i in range(samples + 1):
        a = pi / 2.0 + pi * i / samples
        pts.append((cx - straight + radius * cos(a), cy + radius * sin(a)))
    return pts


def _plate_mesh(length: float, name: str):
    outer = rounded_rect_profile(length, PLATE_HEIGHT, 0.021, corner_segments=10)
    holes = [
        _circle_profile(-0.5 * length + 0.045, 0.0, 0.0085, 20),
        _slot_profile(-0.18 * length, 0.0, 0.070, 0.018, 10),
        _slot_profile(0.18 * length, 0.0, 0.070, 0.018, 10),
        _circle_profile(0.5 * length - 0.045, 0.0, 0.0085, 20),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, PLATE_THICKNESS, center=True),
        name,
    )


def _add_plate(part, *, length: float, direction: float, lane_y: float, material: str, name: str) -> None:
    part.visual(
        _plate_mesh(length, name),
        origin=Origin(
            xyz=(direction * length * 0.5, lane_y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_hinge_barrel(part, *, x: float, lane_y: float, material: str, name: str) -> None:
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LENGTH),
        origin=Origin(xyz=(x, lane_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_pin_head(part, *, x: float, y: float, material: str, name: str) -> None:
    part.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(x, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_fastener_grid(part, *, x_extent: float, y_extent: float, top_z: float, material: str) -> None:
    for ix, xi in enumerate((-0.38 * x_extent, 0.38 * x_extent)):
        for iy, yi in enumerate((-0.34 * y_extent, 0.34 * y_extent)):
            part.visual(
                Cylinder(radius=0.0042, length=0.0032),
                origin=Origin(xyz=(xi, yi, top_z), rpy=(0.0, 0.0, 0.0)),
                material=material,
                name=f"screw_{ix}_{iy}",
            )


def _make_cover(model: ArticulatedObject, parent, *, index: int, x: float, y: float, z: float, length: float):
    cover = model.part(f"cover_{index}")
    cover.visual(
        Box((length, 0.024, 0.006)),
        origin=Origin(),
        material="brushed_access",
        name="cover_plate",
    )
    _add_fastener_grid(cover, x_extent=length, y_extent=0.024, top_z=0.0042, material="dark_screw")
    model.articulation(
        f"cover_{index}_mount",
        ArticulationType.FIXED,
        parent=parent,
        child=cover,
        origin=Origin(xyz=(x, y, z)),
    )
    return cover


def _add_segment(
    part,
    *,
    length: float,
    direction: float,
    lane_y: float,
    plate_name: str,
    material: str,
    accent: str,
) -> None:
    distal_x = direction * length
    _add_plate(part, length=length, direction=direction, lane_y=lane_y, material=material, name=plate_name)
    _add_hinge_barrel(part, x=0.0, lane_y=lane_y, material=accent, name="proximal_barrel")
    _add_hinge_barrel(part, x=distal_x, lane_y=lane_y, material=accent, name="distal_barrel")
    part.visual(
        Box((0.060, 0.020, 0.012)),
        origin=Origin(xyz=(direction * 0.075, lane_y, -0.030)),
        material="black_wear_pad",
        name="proximal_wear_pad",
    )
    part.visual(
        Box((0.060, 0.020, 0.012)),
        origin=Origin(xyz=(direction * (length - 0.075), lane_y, -0.030)),
        material="black_wear_pad",
        name="distal_wear_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_arm_mechanical_study")

    model.material("dark_oxide_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("gunmetal_plate", rgba=(0.31, 0.34, 0.36, 1.0))
    model.material("blued_plate", rgba=(0.18, 0.25, 0.31, 1.0))
    model.material("hardcoat_aluminum", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("brushed_access", rgba=(0.68, 0.70, 0.70, 1.0))
    model.material("pin_head", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("bronze_bushing", rgba=(0.72, 0.52, 0.25, 1.0))
    model.material("black_wear_pad", rgba=(0.025, 0.027, 0.028, 1.0))
    model.material("stop_orange", rgba=(0.95, 0.45, 0.10, 1.0))
    model.material("dark_screw", rgba=(0.015, 0.016, 0.017, 1.0))

    root = model.part("root_frame")
    root.visual(
        Box((0.62, 0.32, 0.030)),
        origin=Origin(xyz=(0.205, 0.0, 0.015)),
        material="dark_oxide_steel",
        name="base_plate",
    )
    root.visual(
        Box((0.46, 0.020, 0.030)),
        origin=Origin(xyz=(0.205, 0.123, 0.059)),
        material="gunmetal_plate",
        name="upper_cradle_rail",
    )
    root.visual(
        Box((0.46, 0.020, 0.030)),
        origin=Origin(xyz=(0.205, -0.123, 0.059)),
        material="gunmetal_plate",
        name="lower_cradle_rail",
    )
    for y in (-0.130, 0.130):
        root.visual(
            Box((0.070, 0.026, 0.128)),
            origin=Origin(xyz=(0.0, y, 0.078)),
            material="gunmetal_plate",
            name=f"root_cheek_{y:+.2f}",
        )
        root.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.050),
            origin=Origin(xyz=(0.0, y * 0.96, ARM_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="bronze_bushing",
            name=f"root_barrel_{y:+.2f}",
        )
    root.visual(
        Box((0.032, 0.040, 0.052)),
        origin=Origin(xyz=(0.468, 0.075, 0.054)),
        material="stop_orange",
        name="stow_stop_tab",
    )
    root.visual(
        Box((0.032, 0.040, 0.052)),
        origin=Origin(xyz=(0.048, -0.025, 0.054)),
        material="stop_orange",
        name="fold_stop_tab",
    )
    root.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.1548, ARM_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_head",
        name="root_pin_head_upper",
    )
    root.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, -0.1548, ARM_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_head",
        name="root_pin_head_lower",
    )
    root.visual(
        Box((0.085, 0.042, 0.060)),
        origin=Origin(xyz=(0.070, -0.142, 0.052)),
        material="gunmetal_plate",
        name="latch_pedestal",
    )
    root.visual(
        Box((0.050, 0.026, 0.100)),
        origin=Origin(xyz=(0.070, -0.161, 0.124)),
        material="gunmetal_plate",
        name="latch_cheek",
    )
    root.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.070, -0.161, ARM_Z + 0.064), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bronze_bushing",
        name="latch_root_barrel",
    )
    root.visual(
        Box((0.078, 0.038, 0.060)),
        origin=Origin(xyz=(0.090, 0.113, 0.074)),
        material="gunmetal_plate",
        name="toggle_pedestal",
    )
    root.visual(
        Box((0.044, 0.024, 0.105)),
        origin=Origin(xyz=(0.090, 0.113, 0.121)),
        material="gunmetal_plate",
        name="toggle_cheek",
    )
    root.visual(
        Cylinder(radius=0.015, length=0.034),
        origin=Origin(xyz=(0.090, 0.1125, ARM_Z + 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bronze_bushing",
        name="toggle_root_barrel",
    )

    section_0 = model.part("section_0")
    section_1 = model.part("section_1")
    section_2 = model.part("section_2")
    section_3 = model.part("section_3")

    lengths = (0.440, 0.365, 0.320, 0.245)
    lanes = (0.075, 0.025, -0.025, -0.075)
    directions = (1.0, -1.0, 1.0, -1.0)
    _add_segment(
        section_0,
        length=lengths[0],
        direction=directions[0],
        lane_y=lanes[0],
        plate_name="web_plate_0",
        material="hardcoat_aluminum",
        accent="bronze_bushing",
    )
    _add_segment(
        section_1,
        length=lengths[1],
        direction=directions[1],
        lane_y=lanes[1],
        plate_name="web_plate_1",
        material="gunmetal_plate",
        accent="bronze_bushing",
    )
    _add_segment(
        section_2,
        length=lengths[2],
        direction=directions[2],
        lane_y=lanes[2],
        plate_name="web_plate_2",
        material="blued_plate",
        accent="bronze_bushing",
    )
    _add_segment(
        section_3,
        length=lengths[3],
        direction=directions[3],
        lane_y=lanes[3],
        plate_name="web_plate_3",
        material="hardcoat_aluminum",
        accent="bronze_bushing",
    )

    # A visible crank lug for the over-center side linkage; the toggle eye nests
    # in the gap between these cheeks when the arm is stowed.
    section_0.visual(
        Box((0.040, 0.008, 0.070)),
        origin=Origin(xyz=(0.360, 0.121, 0.005)),
        material="gunmetal_plate",
        name="toggle_lug_inner",
    )
    section_0.visual(
        Box((0.040, 0.008, 0.070)),
        origin=Origin(xyz=(0.360, 0.149, 0.005)),
        material="gunmetal_plate",
        name="toggle_lug_outer",
    )
    section_0.visual(
        Box((0.044, 0.082, 0.010)),
        origin=Origin(xyz=(0.360, 0.112, -0.026)),
        material="gunmetal_plate",
        name="toggle_lug_bridge",
    )

    for i, (section, length, direction, lane_y) in enumerate(
        zip((section_0, section_1, section_2, section_3), lengths, directions, lanes)
    ):
        _make_cover(
            model,
            section,
            index=i,
            x=direction * length * 0.50,
            y=lane_y,
            z=PLATE_HEIGHT * 0.5 + 0.0024,
            length=min(0.155, length * 0.50),
        )

    end_clevis = section_3
    end_clevis.visual(
        Box((0.080, 0.030, 0.044)),
        origin=Origin(xyz=(-lengths[3] - 0.040, lanes[3], 0.0)),
        material="gunmetal_plate",
        name="clevis_bridge",
    )
    end_clevis.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(-lengths[3] - 0.078, lanes[3], 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bronze_bushing",
        name="terminal_bearing",
    )
    end_clevis.visual(
        Box((0.030, 0.012, 0.060)),
        origin=Origin(xyz=(-lengths[3] - 0.080, lanes[3] + 0.020, 0.0)),
        material="hardcoat_aluminum",
        name="clevis_ear_0",
    )
    end_clevis.visual(
        Box((0.030, 0.012, 0.060)),
        origin=Origin(xyz=(-lengths[3] - 0.080, lanes[3] - 0.020, 0.0)),
        material="hardcoat_aluminum",
        name="clevis_ear_1",
    )

    latch = model.part("packing_latch")
    latch.visual(
        Box((0.270, 0.018, 0.014)),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material="dark_oxide_steel",
        name="latch_bar",
    )
    latch.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bronze_bushing",
        name="latch_pivot_barrel",
    )
    latch.visual(
        Box((0.020, 0.020, 0.052)),
        origin=Origin(xyz=(0.273, 0.0, -0.026)),
        material="stop_orange",
        name="latch_hook",
    )

    toggle = model.part("toggle_link")
    toggle.visual(
        Box((0.270, 0.012, 0.012)),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material="dark_oxide_steel",
        name="toggle_bar",
    )
    toggle.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_head",
        name="toggle_root_eye",
    )
    toggle.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_head",
        name="toggle_arm_eye",
    )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=root,
        child=section_0,
        origin=Origin(xyz=(0.0, 0.0, ARM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=0.0, upper=0.62),
    )
    model.articulation(
        "fold_0",
        ArticulationType.REVOLUTE,
        parent=section_0,
        child=section_1,
        origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=pi),
    )
    model.articulation(
        "fold_1",
        ArticulationType.REVOLUTE,
        parent=section_1,
        child=section_2,
        origin=Origin(xyz=(-lengths[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=pi),
    )
    model.articulation(
        "fold_2",
        ArticulationType.REVOLUTE,
        parent=section_2,
        child=section_3,
        origin=Origin(xyz=(lengths[2], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=pi),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=root,
        child=latch,
        origin=Origin(xyz=(0.070, -0.136, ARM_Z + 0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=0.95),
    )
    model.articulation(
        "toggle_pivot",
        ArticulationType.REVOLUTE,
        parent=root,
        child=toggle,
        origin=Origin(xyz=(0.090, 0.135, ARM_Z + 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-0.50, upper=0.20),
        mimic=Mimic(joint="root_hinge", multiplier=-0.65, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_frame")
    sections = [object_model.get_part(f"section_{i}") for i in range(4)]
    covers = [object_model.get_part(f"cover_{i}") for i in range(4)]
    latch = object_model.get_part("packing_latch")
    toggle = object_model.get_part("toggle_link")
    folds = [object_model.get_articulation(f"fold_{i}") for i in range(3)]
    root_hinge = object_model.get_articulation("root_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.check(
        "multi segment mechanism present",
        len(sections) == 4 and len(folds) == 3 and root_hinge is not None,
        details="Expected four nested rigid sections with three serial fold hinges.",
    )
    ctx.check(
        "fold joints swing through reversal",
        all(j.motion_limits and j.motion_limits.upper >= 3.10 for j in folds),
        details="Each section hinge should be able to reverse from stowed to deployed.",
    )

    ctx.expect_contact(root, sections[0], contact_tol=0.002, name="root hinge knuckles support first section")
    ctx.expect_contact(sections[0], sections[1], contact_tol=0.002, name="first fold hinge knuckles touch")
    ctx.expect_contact(sections[1], sections[2], contact_tol=0.002, name="second fold hinge knuckles touch")
    ctx.expect_contact(sections[2], sections[3], contact_tol=0.002, name="third fold hinge knuckles touch")
    ctx.expect_contact(root, latch, contact_tol=0.002, name="packing latch is carried by root bracket")
    ctx.expect_contact(root, toggle, contact_tol=0.002, name="over center link is carried by root bracket")

    for i, (cover, section) in enumerate(zip(covers, sections)):
        ctx.expect_gap(
            cover,
            section,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0010,
            positive_elem="cover_plate",
            negative_elem=f"web_plate_{i}",
            name=f"cover_{i} seated on section",
        )

    def _span(parts, axis: int):
        boxes = [ctx.part_world_aabb(p) for p in parts]
        boxes = [b for b in boxes if b is not None]
        return min(b[0][axis] for b in boxes), max(b[1][axis] for b in boxes)

    rest_x = _span(sections, 0)
    rest_z = _span(sections, 2)
    ctx.check(
        "stowed arm package remains compact",
        rest_x[1] - rest_x[0] < 0.55 and rest_z[1] - rest_z[0] < 0.09,
        details=f"stowed_x_span={rest_x[1] - rest_x[0]:.3f}, stowed_z_span={rest_z[1] - rest_z[0]:.3f}",
    )

    rest_latch_z = ctx.part_world_aabb(latch)[1][2]
    with ctx.pose(
        {
            root_hinge: 0.55,
            folds[0]: pi,
            folds[1]: pi,
            folds[2]: pi,
            latch_pivot: 0.80,
        }
    ):
        deployed_x = _span(sections, 0)
        deployed_z = _span(sections, 2)
        latch_z = ctx.part_world_aabb(latch)[1][2]
        ctx.check(
            "deployed arm reaches outward",
            deployed_x[1] - deployed_x[0] > 1.15,
            details=f"deployed_x_span={deployed_x[1] - deployed_x[0]:.3f}",
        )
        ctx.check(
            "root hinge lifts deployed chain",
            deployed_z[1] > rest_z[1] + 0.35,
            details=f"rest_z_max={rest_z[1]:.3f}, deployed_z_max={deployed_z[1]:.3f}",
        )
        ctx.check(
            "packing latch swings clear",
            latch_z > rest_latch_z + 0.10,
            details=f"rest_latch_z={rest_latch_z:.3f}, posed_latch_z={latch_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()

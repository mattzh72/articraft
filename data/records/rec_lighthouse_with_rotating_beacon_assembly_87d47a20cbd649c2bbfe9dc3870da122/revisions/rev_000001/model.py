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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_navigation_lighthouse")

    tower_white = model.material("tower_white", rgba=(0.93, 0.94, 0.92, 1.0))
    gallery_red = model.material("gallery_red", rgba=(0.63, 0.12, 0.10, 1.0))
    roof_red = model.material("roof_red", rgba=(0.57, 0.11, 0.09, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.70, 0.85, 0.92, 0.34))
    lantern_frame = model.material("lantern_frame", rgba=(0.20, 0.21, 0.22, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.60, 0.25, 1.0))
    amber_lens = model.material("amber_lens", rgba=(0.93, 0.79, 0.34, 0.62))
    vent_black = model.material("vent_black", rgba=(0.12, 0.12, 0.13, 1.0))

    tower = model.part("tower")
    tower.visual(
        _mesh(
            "tower_body",
            LatheGeometry(
                [
                    (0.0, 0.0),
                    (0.96, 0.0),
                    (0.96, 0.08),
                    (0.93, 0.20),
                    (0.84, 1.10),
                    (0.75, 2.15),
                    (0.68, 3.02),
                    (0.72, 3.18),
                    (0.78, 3.28),
                    (0.0, 3.28),
                ],
                segments=72,
            ),
        ),
        material=tower_white,
        name="tower_body",
    )
    tower.visual(
        Box((0.30, 0.08, 0.64)),
        origin=Origin(xyz=(0.88, 0.0, 0.32)),
        material=gallery_red,
        name="entry_door",
    )
    tower.visual(
        _mesh(
            "gallery_parapet",
            LatheGeometry.from_shell_profiles(
                [(0.85, 0.0), (0.85, 0.12)],
                [(0.61, 0.0), (0.61, 0.12)],
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.28)),
        material=gallery_red,
        name="gallery_parapet",
    )
    tower.visual(
        _mesh(
            "lantern_shell",
            LatheGeometry.from_shell_profiles(
                [(0.58, 0.0), (0.58, 0.64)],
                [(0.53, 0.02), (0.53, 0.62)],
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.28)),
        material=lantern_glass,
        name="lantern_shell",
    )
    for index in range(8):
        angle = index * math.tau / 8.0
        tower.visual(
            Box((0.05, 0.03, 0.64)),
            origin=Origin(
                xyz=(0.555 * math.cos(angle), 0.555 * math.sin(angle), 3.60),
                rpy=(0.0, 0.0, angle),
            ),
            material=lantern_frame,
            name=f"lantern_mullion_{index:02d}",
        )
    tower.visual(
        _mesh(
            "roof_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.62, 0.0),
                    (0.54, 0.06),
                    (0.28, 0.30),
                    (0.20, 0.40),
                    (0.15, 0.48),
                    (0.12, 0.54),
                ],
                [
                    (0.58, 0.02),
                    (0.50, 0.08),
                    (0.24, 0.30),
                    (0.16, 0.40),
                    (0.11, 0.48),
                    (0.08, 0.54),
                ],
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.90)),
        material=roof_red,
        name="roof_shell",
    )
    tower.visual(
        Cylinder(radius=0.11, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 3.38)),
        material=dark_iron,
        name="shaft_pedestal",
    )
    tower.visual(
        Cylinder(radius=0.038, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 3.74)),
        material=dark_iron,
        name="central_shaft",
    )
    tower.visual(
        Cylinder(radius=0.060, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 4.12)),
        material=dark_iron,
        name="shaft_top_guide",
    )
    tower.visual(
        Cylinder(radius=0.085, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 4.47)),
        material=dark_iron,
        name="vent_pivot_base",
    )
    tower.visual(
        Cylinder(radius=0.068, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 4.525)),
        material=dark_iron,
        name="vent_pivot_post",
    )
    tower.inertial = Inertial.from_geometry(
        Box((1.92, 1.92, 4.46)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, 2.23)),
    )

    beacon = model.part("beacon")
    beacon.visual(
        _mesh(
            "beacon_sleeve",
            LatheGeometry.from_shell_profiles(
                [(0.072, -0.12), (0.072, 0.12)],
                [(0.055, -0.11), (0.055, 0.11)],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=brass,
        name="beacon_sleeve",
    )
    beacon.visual(
        Box((0.26, 0.055, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, -0.02)),
        material=dark_iron,
        name="lower_yoke",
    )
    beacon.visual(
        Box((0.20, 0.040, 0.04)),
        origin=Origin(xyz=(0.15, 0.0, 0.08)),
        material=dark_iron,
        name="upper_yoke",
    )
    beacon.visual(
        Box((0.05, 0.060, 0.20)),
        origin=Origin(xyz=(0.24, 0.0, 0.02)),
        material=dark_iron,
        name="housing_bracket",
    )
    beacon.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(0.34, 0.0, 0.03)),
        material=brass,
        name="lamp_house",
    )
    beacon.visual(
        Cylinder(radius=0.045, length=0.02),
        origin=Origin(xyz=(0.43, 0.0, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber_lens,
        name="front_lens",
    )
    beacon.visual(
        Box((0.16, 0.035, 0.035)),
        origin=Origin(xyz=(-0.13, 0.0, 0.00)),
        material=dark_iron,
        name="counterweight_arm",
    )
    beacon.visual(
        Box((0.10, 0.09, 0.08)),
        origin=Origin(xyz=(-0.22, 0.0, 0.00)),
        material=dark_iron,
        name="counterweight",
    )
    beacon.inertial = Inertial.from_geometry(
        Box((0.56, 0.20, 0.28)),
        mass=120.0,
        origin=Origin(xyz=(0.12, 0.0, 0.02)),
    )

    model.articulation(
        "tower_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 3.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5),
    )

    ventilator_cap = model.part("ventilator_cap")
    ventilator_cap.visual(
        _mesh(
            "cap_sleeve",
            LatheGeometry.from_shell_profiles(
                [
                    (0.078, 0.0),
                    (0.078, 0.03),
                    (0.067, 0.05),
                    (0.058, 0.13),
                    (0.18, 0.18),
                    (0.16, 0.225),
                    (0.122, 0.285),
                    (0.074, 0.342),
                    (0.035, 0.375),
                ],
                [
                    (0.060, 0.0),
                    (0.060, 0.026),
                    (0.051, 0.046),
                    (0.044, 0.126),
                    (0.146, 0.186),
                    (0.131, 0.225),
                    (0.099, 0.277),
                    (0.058, 0.327),
                    (0.018, 0.355),
                ],
                segments=56,
                start_cap="flat",
                end_cap="round",
            ),
        ),
        material=vent_black,
        name="cap_sleeve",
    )
    ventilator_cap.visual(
        Box((0.18, 0.03, 0.04)),
        origin=Origin(xyz=(-0.09, 0.0, 0.25)),
        material=vent_black,
        name="fin_mount",
    )
    ventilator_cap.visual(
        Box((0.12, 0.012, 0.09)),
        origin=Origin(xyz=(-0.18, 0.0, 0.29)),
        material=vent_black,
        name="cap_tail_fin",
    )
    ventilator_cap.inertial = Inertial.from_geometry(
        Box((0.40, 0.24, 0.39)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    model.articulation(
        "roof_to_ventilator_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=ventilator_cap,
        origin=Origin(xyz=(0.0, 0.0, 4.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    ventilator_cap = object_model.get_part("ventilator_cap")
    beacon_spin = object_model.get_articulation("tower_to_beacon")
    cap_spin = object_model.get_articulation("roof_to_ventilator_cap")

    for part_name in ("tower", "beacon", "ventilator_cap"):
        ctx.check(
            f"part {part_name} exists",
            object_model.get_part(part_name) is not None,
            details=f"missing part {part_name}",
        )

    ctx.expect_overlap(
        beacon,
        tower,
        axes="xy",
        elem_a="beacon_sleeve",
        elem_b="central_shaft",
        min_overlap=0.07,
        name="beacon sleeve is centered on the main shaft",
    )
    ctx.expect_overlap(
        ventilator_cap,
        tower,
        axes="xy",
        elem_a="cap_sleeve",
        elem_b="vent_pivot_post",
        min_overlap=0.06,
        name="ventilator cap sleeve is centered on the roof pivot",
    )
    ctx.expect_contact(
        ventilator_cap,
        tower,
        elem_a="cap_sleeve",
        elem_b="vent_pivot_post",
        name="ventilator cap sleeve is seated on the pivot post",
    )
    ctx.expect_gap(
        ventilator_cap,
        tower,
        axis="z",
        positive_elem="cap_sleeve",
        negative_elem="roof_shell",
        min_gap=0.10,
        name="ventilator cap clears the roof shell",
    )
    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="lamp_house",
        outer_elem="lantern_shell",
        margin=0.0,
        name="beacon lamp stays within the lantern room at rest",
    )

    lamp_rest = _aabb_center(ctx.part_element_world_aabb(beacon, elem="lamp_house"))
    with ctx.pose({beacon_spin: math.pi / 2.0}):
        ctx.expect_within(
            beacon,
            tower,
            axes="xy",
            inner_elem="lamp_house",
            outer_elem="lantern_shell",
            margin=0.0,
            name="beacon lamp stays within the lantern room while rotating",
        )
        lamp_quarter = _aabb_center(ctx.part_element_world_aabb(beacon, elem="lamp_house"))
    beacon_rotates = (
        lamp_rest is not None
        and lamp_quarter is not None
        and abs(lamp_rest[0]) > 0.20
        and abs(lamp_quarter[1]) > 0.20
        and abs(lamp_rest[2] - lamp_quarter[2]) < 0.02
    )
    ctx.check(
        "beacon rotates about the vertical shaft",
        beacon_rotates,
        details=f"rest={lamp_rest}, quarter_turn={lamp_quarter}",
    )

    fin_rest = _aabb_center(ctx.part_element_world_aabb(ventilator_cap, elem="cap_tail_fin"))
    with ctx.pose({cap_spin: math.pi / 2.0}):
        fin_quarter = _aabb_center(ctx.part_element_world_aabb(ventilator_cap, elem="cap_tail_fin"))
    cap_rotates = (
        fin_rest is not None
        and fin_quarter is not None
        and abs(fin_rest[0]) > 0.06
        and abs(fin_quarter[1]) > 0.06
        and abs(fin_rest[2] - fin_quarter[2]) < 0.02
    )
    ctx.check(
        "ventilator cap rotates on its own vertical pivot",
        cap_rotates,
        details=f"rest={fin_rest}, quarter_turn={fin_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

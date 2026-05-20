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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_cap_mesh():
    outer_profile = [
        (0.54, 0.00),
        (0.49, 0.08),
        (0.505, 0.085),
        (0.44, 0.17),
        (0.455, 0.175),
        (0.37, 0.29),
        (0.385, 0.295),
        (0.28, 0.43),
        (0.295, 0.435),
        (0.18, 0.58),
        (0.12, 0.68),
        (0.05, 0.76),
    ]
    inner_profile = [
        (0.24, 0.00),
        (0.23, 0.08),
        (0.22, 0.17),
        (0.19, 0.29),
        (0.15, 0.43),
        (0.10, 0.58),
        (0.05, 0.71),
        (0.00, 0.76),
    ]

    cap = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    cap.merge(
        CylinderGeometry(radius=0.028, height=0.09, radial_segments=24).translate(0.0, 0.0, 0.805)
    )
    cap.merge(SphereGeometry(radius=0.045).translate(0.0, 0.0, 0.87))
    return cap


def _build_bell_mesh():
    outer_profile = [
        (0.275, -0.56),
        (0.268, -0.51),
        (0.250, -0.43),
        (0.216, -0.31),
        (0.180, -0.20),
        (0.148, -0.11),
        (0.121, -0.05),
        (0.102, -0.015),
    ]
    inner_profile = [
        (0.236, -0.56),
        (0.229, -0.51),
        (0.213, -0.43),
        (0.182, -0.31),
        (0.151, -0.20),
        (0.126, -0.11),
        (0.101, -0.05),
        (0.084, -0.015),
    ]

    bell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )
    bell.merge(
        TorusGeometry(radius=0.248, tube=0.015, radial_segments=18, tubular_segments=56).translate(
            0.0,
            0.0,
            -0.552,
        )
    )
    bell.merge(CylinderGeometry(radius=0.104, height=0.035, radial_segments=32).translate(0.0, 0.0, -0.03))
    return bell


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_bell_tower")

    timber = model.material("timber", rgba=(0.57, 0.48, 0.38, 1.0))
    shingles = model.material("shingles", rgba=(0.41, 0.35, 0.29, 1.0))
    iron = model.material("iron", rgba=(0.16, 0.17, 0.18, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.18, length=3.35),
        origin=Origin(xyz=(0.0, 0.0, 1.675)),
        material=timber,
        name="main_post",
    )
    tower.visual(
        Box((1.45, 0.18, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 3.01)),
        material=timber,
        name="crossbeam",
    )
    tower.visual(
        Box((0.30, 0.64, 0.10)),
        origin=Origin(xyz=(0.0, 0.40, 2.97)),
        material=timber,
        name="headstock_block",
    )
    tower.visual(
        Box((0.05, 0.18, 0.09)),
        origin=Origin(xyz=(-0.1375, 0.80, 2.96)),
        material=iron,
        name="hanger_cheek_left",
    )
    tower.visual(
        Box((0.05, 0.18, 0.09)),
        origin=Origin(xyz=(0.1375, 0.80, 2.96)),
        material=iron,
        name="hanger_cheek_right",
    )
    tower.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(-0.1375, 0.85, 2.86)),
        material=iron,
        name="cleat_left",
    )
    tower.visual(
        Box((0.05, 0.10, 0.12)),
        origin=Origin(xyz=(0.1375, 0.85, 2.86)),
        material=iron,
        name="cleat_right",
    )
    tower.visual(
        Box((0.11, 0.11, 0.66)),
        origin=Origin(xyz=(0.34, 0.0, 2.71), rpy=(0.0, math.radians(40.0), 0.0)),
        material=timber,
        name="brace_starboard",
    )
    tower.visual(
        Box((0.11, 0.11, 0.66)),
        origin=Origin(xyz=(-0.34, 0.0, 2.71), rpy=(0.0, math.radians(-40.0), 0.0)),
        material=timber,
        name="brace_port",
    )
    tower.visual(
        Cylinder(radius=0.24, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 3.13)),
        material=timber,
        name="cap_collar",
    )
    tower.visual(
        _save_mesh("cap_shell", _build_cap_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 3.13)),
        material=shingles,
        name="cap_shell",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.54, length=4.02),
        mass=380.0,
        origin=Origin(xyz=(0.0, 0.0, 2.01)),
    )

    bell = model.part("bell")
    bell.visual(
        _save_mesh("bell_shell", _build_bell_mesh()),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=iron,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.050, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=iron,
        name="crown_stem",
    )
    bell.visual(
        Box((0.19, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=iron,
        name="bell_headstock",
    )
    bell.visual(
        Box((0.038, 0.03, 0.20)),
        origin=Origin(xyz=(-0.075, 0.0, -0.10)),
        material=iron,
        name="hanger_left",
    )
    bell.visual(
        Box((0.038, 0.03, 0.20)),
        origin=Origin(xyz=(0.075, 0.0, -0.10)),
        material=iron,
        name="hanger_right",
    )
    bell.visual(
        Cylinder(radius=0.024, length=0.225),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="trunnion_axle",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.28, length=0.62),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.85, 2.86)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=650.0,
            velocity=1.4,
            lower=-0.75,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("tower_to_bell")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            bell,
            tower,
            elem_a="trunnion_axle",
            elem_b="cleat_left",
            name="left trunnion bears on the left cleat",
        )
        ctx.expect_contact(
            bell,
            tower,
            elem_a="trunnion_axle",
            elem_b="cleat_right",
            name="right trunnion bears on the right cleat",
        )
        ctx.expect_gap(
            tower,
            bell,
            axis="z",
            positive_elem="crossbeam",
            negative_elem="bell_shell",
            min_gap=0.05,
            max_gap=0.20,
            name="bell shell hangs below the crossbeam",
        )
        ctx.expect_gap(
            bell,
            tower,
            axis="y",
            positive_elem="bell_shell",
            negative_elem="main_post",
            min_gap=0.18,
            name="bell hangs clear of the timber post",
        )
        ctx.expect_overlap(
            tower,
            bell,
            axes="x",
            elem_a="crossbeam",
            elem_b="bell_shell",
            min_overlap=0.45,
            name="bell remains centered under the beam span",
        )

    lower = hinge.motion_limits.lower if hinge.motion_limits is not None else None
    upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None

    rest_aabb = None
    swung_aabb = None

    with ctx.pose({hinge: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")

    if lower is not None:
        with ctx.pose({hinge: lower}):
            ctx.expect_gap(
                bell,
                tower,
                axis="y",
                positive_elem="bell_shell",
                negative_elem="main_post",
                min_gap=0.02,
                name="aft swing still clears the post",
            )

    if upper is not None:
        with ctx.pose({hinge: upper}):
            ctx.expect_gap(
                tower,
                bell,
                axis="z",
                positive_elem="crossbeam",
                negative_elem="bell_shell",
                min_gap=0.03,
                name="swung bell still clears the crossbeam",
            )
            swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")

    rest_center = _aabb_center(rest_aabb)
    swung_center = _aabb_center(swung_aabb)
    ctx.check(
        "positive bell rotation swings the bell toward +y",
        rest_center is not None
        and swung_center is not None
        and swung_center[1] > rest_center[1] + 0.10,
        details=f"rest_center={rest_center}, swung_center={swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

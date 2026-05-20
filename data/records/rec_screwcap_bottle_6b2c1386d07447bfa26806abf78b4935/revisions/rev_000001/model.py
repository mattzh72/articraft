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
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    clear_pet = model.material("clear_pet", rgba=(0.84, 0.92, 0.98, 0.34))
    cap_blue = model.material("cap_blue", rgba=(0.14, 0.39, 0.78, 1.0))
    cap_shadow = model.material("cap_shadow", rgba=(0.09, 0.19, 0.39, 1.0))

    lower_shell = _mesh(
        "lower_body_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.012, 0.000),
                (0.028, 0.004),
                (0.035, 0.012),
                (0.0385, 0.038),
                (0.0395, 0.082),
                (0.0385, 0.108),
            ],
            [
                (0.000, 0.0065),
                (0.020, 0.010),
                (0.0332, 0.017),
                (0.0367, 0.040),
                (0.0375, 0.100),
                (0.0367, 0.108),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    middle_shell = _mesh(
        "middle_body_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0385, 0.000),
                (0.0400, 0.030),
                (0.0400, 0.080),
                (0.0375, 0.112),
            ],
            [
                (0.0367, 0.000),
                (0.0382, 0.030),
                (0.0382, 0.080),
                (0.0358, 0.112),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    neck_shell = _mesh(
        "neck_finish_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0375, 0.000),
                (0.0345, 0.010),
                (0.0260, 0.024),
                (0.0180, 0.036),
                (0.0152, 0.056),
            ],
            [
                (0.0358, 0.000),
                (0.0328, 0.010),
                (0.0242, 0.024),
                (0.0132, 0.036),
                (0.0132, 0.056),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    cap_shell = _mesh(
        "cap_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0220, 0.0000),
                (0.0218, 0.0030),
                (0.0216, 0.0240),
                (0.0188, 0.0310),
                (0.0000, 0.0330),
            ],
            [
                (0.0194, 0.0018),
                (0.0194, 0.0230),
                (0.0132, 0.0270),
                (0.0056, 0.0296),
                (0.0000, 0.0310),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    base_foot_ring = _mesh(
        "base_foot_ring",
        TorusGeometry(radius=0.0345, tube=0.0020, radial_segments=16, tubular_segments=56),
    )
    label_band_lower = _mesh(
        "label_band_lower",
        TorusGeometry(radius=0.0390, tube=0.0012, radial_segments=16, tubular_segments=56),
    )
    label_band_upper = _mesh(
        "label_band_upper",
        TorusGeometry(radius=0.0386, tube=0.0010, radial_segments=16, tubular_segments=56),
    )

    lower_body = model.part("lower_body")
    lower_body.visual(lower_shell, material=clear_pet, name="lower_shell")
    lower_body.visual(
        base_foot_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=clear_pet,
        name="base_foot_ring",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((0.082, 0.082, 0.108)),
        mass=0.022,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )

    middle_body = model.part("middle_body")
    middle_body.visual(middle_shell, material=clear_pet, name="middle_shell")
    middle_body.visual(
        label_band_lower,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=clear_pet,
        name="label_band_lower",
    )
    middle_body.visual(
        label_band_upper,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=clear_pet,
        name="label_band_upper",
    )
    middle_body.inertial = Inertial.from_geometry(
        Box((0.082, 0.082, 0.112)),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    neck_finish = model.part("neck_finish")
    neck_finish.visual(neck_shell, material=clear_pet, name="neck_shell")
    neck_finish.visual(
        Cylinder(radius=0.0226, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=clear_pet,
        name="support_bead",
    )
    for index, z in enumerate((0.0350, 0.0415, 0.0480)):
        neck_finish.visual(
            Cylinder(radius=0.0168 - 0.0005 * index, length=0.0022),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=clear_pet,
            name=f"thread_turn_{index}",
        )
    neck_finish.visual(
        Cylinder(radius=0.0155, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0548)),
        material=clear_pet,
        name="thread_band",
    )
    neck_finish.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.056),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    cap = model.part("cap")
    cap.visual(cap_shell, material=cap_blue, name="cap_shell")
    cap.visual(
        Cylinder(radius=0.0120, length=0.0082),
        origin=Origin(xyz=(0.0, 0.0, 0.0271)),
        material=cap_shadow,
        name="seal_plug",
    )
    cap.visual(
        Cylinder(radius=0.0142, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0236)),
        material=cap_shadow,
        name="seal_flange",
    )
    rib_count = 24
    for index in range(rib_count):
        angle = (2.0 * math.pi * index) / rib_count
        cap.visual(
            Box((0.0014, 0.0030, 0.0240)),
            origin=Origin(
                xyz=(0.0221 * math.cos(angle), 0.0221 * math.sin(angle), 0.0205),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_shadow if index % 2 else cap_blue,
            name=f"grip_rib_{index:02d}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.033),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
    )

    model.articulation(
        "lower_to_middle",
        ArticulationType.FIXED,
        parent=lower_body,
        child=middle_body,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )
    model.articulation(
        "middle_to_neck",
        ArticulationType.FIXED,
        parent=middle_body,
        child=neck_finish,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
    )
    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=neck_finish,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    middle_body = object_model.get_part("middle_body")
    neck_finish = object_model.get_part("neck_finish")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    with ctx.pose({cap_spin: 0.0}):
        ctx.expect_gap(
            middle_body,
            lower_body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0005,
            positive_elem="middle_shell",
            negative_elem="lower_shell",
            name="middle section seats on lower section",
        )
        ctx.expect_overlap(
            middle_body,
            lower_body,
            axes="xy",
            min_overlap=0.072,
            elem_a="middle_shell",
            elem_b="lower_shell",
            name="lower and middle sections stay coaxial",
        )
        ctx.expect_gap(
            neck_finish,
            middle_body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0005,
            positive_elem="neck_shell",
            negative_elem="middle_shell",
            name="neck section seats on shoulder section",
        )
        ctx.expect_overlap(
            neck_finish,
            cap,
            axes="xy",
            min_overlap=0.030,
            elem_a="thread_band",
            elem_b="cap_shell",
            name="cap skirt surrounds neck finish",
        )
        ctx.expect_overlap(
            neck_finish,
            cap,
            axes="z",
            min_overlap=0.020,
            elem_a="neck_shell",
            elem_b="cap_shell",
            name="cap covers the threaded neck",
        )

    with ctx.pose({cap_spin: 2.3}):
        ctx.expect_overlap(
            neck_finish,
            cap,
            axes="xy",
            min_overlap=0.030,
            elem_a="thread_band",
            elem_b="cap_shell",
            name="rotated cap remains centered on neck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

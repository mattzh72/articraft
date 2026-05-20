from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _axial_shell_mesh(
    filename: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    segments: int = 64,
    start_cap: str = "flat",
    end_cap: str = "flat",
    lip_samples: int = 8,
):
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap=start_cap,
        end_cap=end_cap,
        lip_samples=lip_samples,
    ).rotate_y(math.pi / 2.0)
    return _save_mesh(filename, geometry)


def _ring_mesh(
    filename: str,
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    segments: int = 56,
):
    half = 0.5 * length
    return _axial_shell_mesh(
        filename,
        outer_profile=[(outer_radius, -half), (outer_radius, half)],
        inner_profile=[(inner_radius, -half), (inner_radius, half)],
        segments=segments,
    )


def _radial_box_origin(
    *,
    x: float,
    radius: float,
    angle: float,
) -> Origin:
    return Origin(
        xyz=(x, radius * math.cos(angle), radius * math.sin(angle)),
        rpy=(angle, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_prime_lens", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    glass = model.material("front_glass", rgba=(0.54, 0.68, 0.78, 0.38))
    engraving_white = model.material("engraving_white", rgba=(0.88, 0.88, 0.82, 1.0))

    body = model.part("lens_body")
    body.visual(
        Cylinder(radius=0.036, length=0.045),
        origin=Origin(xyz=(0.0305, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_tube",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.033),
        origin=Origin(xyz=(0.0685, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="iris_seat",
    )
    body.visual(
        Cylinder(radius=0.0392, length=0.003),
        origin=Origin(xyz=(0.0535, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="iris_rear_collar",
    )
    body.visual(
        Cylinder(radius=0.0392, length=0.003),
        origin=Origin(xyz=(0.0835, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="iris_front_collar",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.003),
        origin=Origin(xyz=(0.0855, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="bridge_tube",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.063),
        origin=Origin(xyz=(0.1175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="focus_seat",
    )
    body.visual(
        Cylinder(radius=0.0402, length=0.003),
        origin=Origin(xyz=(0.0875, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="focus_rear_collar",
    )
    body.visual(
        Cylinder(radius=0.0402, length=0.003),
        origin=Origin(xyz=(0.1475, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="focus_front_collar",
    )
    body.visual(
        Cylinder(radius=0.043, length=0.009),
        origin=Origin(xyz=(0.1525, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_transition",
    )
    body.visual(
        _axial_shell_mesh(
            "front_housing_shell.obj",
            outer_profile=[
                (0.043, -0.022),
                (0.046, -0.012),
                (0.050, 0.000),
                (0.054, 0.014),
                (0.056, 0.022),
            ],
            inner_profile=[
                (0.034, -0.022),
                (0.036, -0.010),
                (0.039, 0.002),
                (0.043, 0.016),
                (0.046, 0.022),
            ],
            segments=72,
        ),
        origin=Origin(xyz=(0.1785, 0.0, 0.0)),
        material=matte_black,
        name="front_housing_shell",
    )
    body.visual(
        _ring_mesh(
            "front_flange_ring.obj",
            inner_radius=0.049,
            outer_radius=0.058,
            length=0.008,
            segments=72,
        ),
        origin=Origin(xyz=(0.204, 0.0, 0.0)),
        material=dark_metal,
        name="front_flange",
    )
    body.visual(
        _ring_mesh(
            "front_inner_baffle.obj",
            inner_radius=0.032,
            outer_radius=0.041,
            length=0.018,
            segments=60,
        ),
        origin=Origin(xyz=(0.166, 0.0, 0.0)),
        material=dark_metal,
        name="inner_baffle",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        body.visual(
            Box((0.005, 0.003, 0.010)),
            origin=_radial_box_origin(x=0.203, radius=0.0575, angle=angle),
            material=dark_metal,
            name=f"bayonet_tab_{index:02d}",
        )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.208),
        mass=1.6,
        origin=Origin(xyz=(0.104, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rear_mount = model.part("rear_mount")
    rear_mount.visual(
        _ring_mesh(
            "rear_mount_ring.obj",
            inner_radius=0.0365,
            outer_radius=0.041,
            length=0.016,
            segments=64,
        ),
        origin=Origin(),
        material=dark_metal,
        name="mount_shell",
    )
    rear_mount.visual(
        Box((0.006, 0.006, 0.014)),
        origin=Origin(xyz=(0.003, 0.041, 0.0)),
        material=dark_metal,
        name="lever_pad",
    )
    rear_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.016),
        mass=0.18,
        origin=Origin(),
    )

    lock_lever = model.part("lock_lever")
    lock_lever.visual(
        Box((0.007, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_metal,
        name="lever_base",
    )
    lock_lever.visual(
        Box((0.016, 0.006, 0.004)),
        origin=Origin(xyz=(0.009, 0.002, 0.0), rpy=(0.0, 0.0, -0.18)),
        material=satin_metal,
        name="lever_paddle",
    )
    lock_lever.visual(
        Box((0.008, 0.004, 0.003)),
        origin=Origin(xyz=(0.015, 0.0035, 0.0), rpy=(0.0, 0.0, -0.18)),
        material=satin_metal,
        name="lever_tip",
    )
    lock_lever.inertial = Inertial.from_geometry(
        Box((0.024, 0.016, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(0.010, 0.005, 0.0)),
    )

    iris_ring = model.part("iris_ring")
    iris_ring.visual(
        _ring_mesh(
            "iris_band_ring.obj",
            inner_radius=0.0392,
            outer_radius=0.043,
            length=0.026,
            segments=64,
        ),
        origin=Origin(),
        material=matte_black,
        name="iris_band",
    )
    for index, x_pos in enumerate((-0.006, 0.006)):
        iris_ring.visual(
            Box((0.005, 0.0012, 0.0045)),
            origin=Origin(xyz=(x_pos, 0.0430, 0.0)),
            material=engraving_white,
            name=f"iris_mark_{index:02d}",
        )
    iris_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.026),
        mass=0.08,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _ring_mesh(
            "focus_band_ring.obj",
            inner_radius=0.0402,
            outer_radius=0.045,
            length=0.062,
            segments=72,
        ),
        origin=Origin(),
        material=rubber_black,
        name="focus_band",
    )
    for index, x_pos in enumerate((-0.020, -0.008, 0.004, 0.016)):
        focus_ring.visual(
            Box((0.006, 0.0012, 0.0055)),
            origin=Origin(xyz=(x_pos, 0.0452, 0.0)),
            material=engraving_white,
            name=f"focus_mark_{index:02d}",
        )
    for index in range(28):
        angle = (2.0 * math.pi * index) / 28.0
        focus_ring.visual(
            Box((0.059, 0.0028, 0.0042)),
            origin=_radial_box_origin(x=0.0, radius=0.0457, angle=angle),
            material=rubber_black,
            name=f"grip_ridge_{index:02d}",
        )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.047, length=0.062),
        mass=0.14,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    front_optics = model.part("front_optics")
    front_optics.visual(
        _ring_mesh(
            "front_retainer_ring.obj",
            inner_radius=0.0398,
            outer_radius=0.043,
            length=0.004,
            segments=72,
        ),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
        material=dark_metal,
        name="retainer_ring",
    )
    front_optics.visual(
        _ring_mesh(
            "front_pressure_ring.obj",
            inner_radius=0.0394,
            outer_radius=0.0398,
            length=0.006,
            segments=72,
        ),
        origin=Origin(xyz=(0.0035, 0.0, 0.0)),
        material=dark_metal,
        name="pressure_ring",
    )
    front_optics.visual(
        Cylinder(radius=0.0394, length=0.008),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    front_optics.visual(
        Cylinder(radius=0.035, length=0.004),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="rear_glass_group",
    )
    front_optics.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.013),
        mass=0.11,
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    shade = model.part("lens_shade")
    shade.visual(
        _ring_mesh(
            "shade_bayonet_collar.obj",
            inner_radius=0.058,
            outer_radius=0.060,
            length=0.010,
            segments=72,
        ),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=matte_black,
        name="bayonet_collar",
    )
    shade.visual(
        _axial_shell_mesh(
            "shade_hood_shell.obj",
            outer_profile=[
                (0.060, -0.028),
                (0.061, -0.018),
                (0.064, 0.002),
                (0.067, 0.018),
                (0.068, 0.028),
            ],
            inner_profile=[
                (0.056, -0.028),
                (0.057, -0.018),
                (0.059, 0.002),
                (0.061, 0.018),
                (0.062, 0.028),
            ],
            segments=72,
        ),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=matte_black,
        name="hood_shell",
    )
    shade.visual(
        _ring_mesh(
            "shade_lug_ring.obj",
            inner_radius=0.058,
            outer_radius=0.060,
            length=0.010,
            segments=72,
        ),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=matte_black,
        name="lug_ring",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        shade.visual(
            Box((0.006, 0.003, 0.010)),
            origin=_radial_box_origin(x=-0.005, radius=0.0575, angle=angle),
            material=matte_black,
            name=f"shade_bayonet_stop_{index:02d}",
        )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.068, length=0.064),
        mass=0.13,
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "rear_mount_attach",
        ArticulationType.FIXED,
        parent=body,
        child=rear_mount,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
    )
    model.articulation(
        "lock_lever_attach",
        ArticulationType.FIXED,
        parent=rear_mount,
        child=lock_lever,
        origin=Origin(xyz=(0.003, 0.041, 0.0)),
    )
    model.articulation(
        "iris_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=iris_ring,
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=7.0),
    )
    model.articulation(
        "focus_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.117, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=7.0),
    )
    model.articulation(
        "front_optics_mount",
        ArticulationType.FIXED,
        parent=body,
        child=front_optics,
        origin=Origin(xyz=(0.172, 0.0, 0.0)),
    )
    model.articulation(
        "shade_mount",
        ArticulationType.FIXED,
        parent=body,
        child=shade,
        origin=Origin(xyz=(0.208, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("lens_body")
    rear_mount = object_model.get_part("rear_mount")
    lock_lever = object_model.get_part("lock_lever")
    iris_ring = object_model.get_part("iris_ring")
    focus_ring = object_model.get_part("focus_ring")
    front_optics = object_model.get_part("front_optics")
    shade = object_model.get_part("lens_shade")

    iris_spin = object_model.get_articulation("iris_spin")
    focus_spin = object_model.get_articulation("focus_spin")

    rear_tube = body.get_visual("rear_tube")
    iris_seat = body.get_visual("iris_seat")
    focus_seat = body.get_visual("focus_seat")
    front_housing = body.get_visual("front_housing_shell")
    front_flange = body.get_visual("front_flange")

    mount_shell = rear_mount.get_visual("mount_shell")
    lever_pad = rear_mount.get_visual("lever_pad")
    lever_base = lock_lever.get_visual("lever_base")
    lever_paddle = lock_lever.get_visual("lever_paddle")
    iris_band = iris_ring.get_visual("iris_band")
    focus_band = focus_ring.get_visual("focus_band")
    front_glass = front_optics.get_visual("front_glass")
    retainer_ring = front_optics.get_visual("retainer_ring")
    shade_collar = shade.get_visual("bayonet_collar")
    hood_shell = shade.get_visual("hood_shell")
    bayonet_tab = body.get_visual("bayonet_tab_00")
    shade_stop = shade.get_visual("shade_bayonet_stop_00")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(iris_ring, body, reason="iris sleeve nests on thin bearing collars around the inner barrel")
    ctx.allow_overlap(focus_ring, body, reason="focus sleeve nests on thin bearing collars around the inner barrel")

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.06)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(body, rear_mount, axes="yz", inner_elem=rear_tube, outer_elem=mount_shell)
    ctx.expect_contact(lock_lever, rear_mount, elem_a=lever_base, elem_b=lever_pad)
    ctx.expect_overlap(lock_lever, rear_mount, axes="xz", min_overlap=0.00005, elem_a=lever_paddle, elem_b=mount_shell)

    ctx.expect_within(body, iris_ring, axes="yz", inner_elem=iris_seat, outer_elem=iris_band)
    ctx.expect_within(body, focus_ring, axes="yz", inner_elem=focus_seat, outer_elem=focus_band)
    ctx.expect_contact(iris_ring, body)
    ctx.expect_contact(focus_ring, body)
    ctx.expect_origin_distance(iris_ring, body, axes="yz", max_dist=0.001)
    ctx.expect_origin_distance(focus_ring, body, axes="yz", max_dist=0.001)
    ctx.expect_within(iris_ring, focus_ring, axes="yz", inner_elem=iris_band, outer_elem=focus_band)
    ctx.expect_gap(
        focus_ring,
        iris_ring,
        axis="x",
        min_gap=0.003,
        max_gap=0.006,
        positive_elem=focus_band,
        negative_elem=iris_band,
    )
    ctx.expect_within(focus_ring, body, axes="yz", inner_elem=focus_band, outer_elem=front_housing)

    ctx.expect_within(front_optics, body, axes="yz", inner_elem=retainer_ring, outer_elem=front_housing)
    ctx.expect_gap(
        body,
        front_optics,
        axis="x",
        min_gap=0.015,
        max_gap=0.025,
        positive_elem=front_flange,
        negative_elem=front_glass,
    )

    ctx.expect_within(body, shade, axes="yz", inner_elem=front_flange, outer_elem=shade_collar)
    ctx.expect_contact(shade, body, elem_a=shade_collar, elem_b=front_flange)
    ctx.expect_contact(shade, body, elem_a=shade_stop, elem_b=bayonet_tab)
    ctx.expect_within(body, shade, axes="yz", inner_elem=front_housing, outer_elem=hood_shell)

    with ctx.pose({focus_spin: math.pi / 1.8, iris_spin: -math.pi / 3.0}):
        ctx.expect_within(body, focus_ring, axes="yz", inner_elem=focus_seat, outer_elem=focus_band)
        ctx.expect_within(body, iris_ring, axes="yz", inner_elem=iris_seat, outer_elem=iris_band)
        ctx.expect_gap(
            focus_ring,
            iris_ring,
            axis="x",
            min_gap=0.003,
            max_gap=0.006,
            positive_elem=focus_band,
            negative_elem=iris_band,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

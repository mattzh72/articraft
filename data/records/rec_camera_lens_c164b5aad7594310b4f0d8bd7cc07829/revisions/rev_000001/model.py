from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_meshes(meshes: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for mesh in meshes:
        merged.merge(mesh)
    return merged


def _x_lathe(mesh: MeshGeometry) -> MeshGeometry:
    """Turn a Z-axis lathe mesh into an X-axis lens part."""
    return mesh.rotate_y(math.pi / 2.0)


def _shell_x(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 88,
) -> MeshGeometry:
    return _x_lathe(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
            lip_samples=5,
        )
    )


def _simple_ring_x(
    x_min: float,
    x_max: float,
    outer_radius: float,
    inner_radius: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    return _shell_x(
        [(outer_radius, x_min), (outer_radius, x_max)],
        [(inner_radius, x_min), (inner_radius, x_max)],
        segments=segments,
    )


def _radial_box_ribs(
    *,
    x_center: float,
    length: float,
    radius: float,
    count: int,
    tangential_width: float,
    radial_height: float,
    phase: float = 0.0,
) -> MeshGeometry:
    ribs = MeshGeometry()
    for index in range(count):
        angle = phase + math.tau * index / count
        rib = BoxGeometry((length, tangential_width, radial_height))
        # Slightly sink the rib into the ring so the mesh is visibly and
        # geometrically mounted to the cylindrical sleeve.
        rib.translate(x_center, 0.0, radius + radial_height * 0.42)
        rib.rotate_x(angle)
        ribs.merge(rib)
    return ribs


def _top_rib(
    *,
    x_center: float,
    length: float,
    radius: float,
    width: float,
    height: float,
) -> MeshGeometry:
    rib = BoxGeometry((length, width, height))
    rib.translate(x_center, 0.0, radius + height * 0.40)
    return rib


def _build_barrel_mesh() -> MeshGeometry:
    sleeve = _shell_x(
        [
            (0.080, -0.215),
            (0.088, -0.203),
            (0.089, -0.126),
            (0.095, -0.116),
            (0.090, -0.106),
            (0.090, -0.004),
            (0.090, 0.010),
            (0.096, 0.052),
            (0.087, 0.064),
            (0.087, 0.070),
            (0.087, 0.078),
            (0.087, 0.136),
            (0.087, 0.146),
            (0.087, 0.150),
            (0.080, 0.166),
        ],
        [
            (0.068, -0.216),
            (0.070, -0.202),
            (0.070, -0.125),
            (0.071, -0.106),
            (0.071, 0.044),
            (0.071, 0.064),
            (0.070, 0.150),
            (0.066, 0.168),
        ],
        segments=112,
    )
    rear_pl_plate = _shell_x(
        [
            (0.090, -0.252),
            (0.092, -0.236),
            (0.092, -0.213),
            (0.082, -0.203),
        ],
        [
            (0.046, -0.254),
            (0.046, -0.236),
            (0.052, -0.213),
            (0.060, -0.203),
        ],
        segments=96,
    )
    bayonet_lugs = _radial_box_ribs(
        x_center=-0.235,
        length=0.020,
        radius=0.086,
        count=4,
        tangential_width=0.038,
        radial_height=0.012,
        phase=math.pi / 4.0,
    )
    rear_key = _top_rib(x_center=-0.252, length=0.020, radius=0.052, width=0.012, height=0.010)
    return _merge_meshes([sleeve, rear_pl_plate, bayonet_lugs, rear_key])


def _build_zoom_ring_mesh() -> MeshGeometry:
    ring = _shell_x(
        [
            (0.103, -0.056),
            (0.108, -0.046),
            (0.108, 0.046),
            (0.103, 0.056),
        ],
        [
            (0.096, -0.058),
            (0.096, 0.058),
        ],
        segments=112,
    )
    teeth = _radial_box_ribs(
        x_center=0.0,
        length=0.090,
        radius=0.107,
        count=38,
        tangential_width=0.0044,
        radial_height=0.006,
    )
    t_handle_stem = _top_rib(x_center=0.030, length=0.034, radius=0.113, width=0.014, height=0.011)
    t_handle_cross = BoxGeometry((0.010, 0.046, 0.012))
    t_handle_cross.translate(0.046, 0.0, 0.122)
    return _merge_meshes([ring, teeth, t_handle_stem, t_handle_cross])


def _build_iris_ring_mesh() -> MeshGeometry:
    ring = _shell_x(
        [
            (0.096, -0.030),
            (0.100, -0.022),
            (0.100, 0.022),
            (0.096, 0.030),
        ],
        [(0.091, -0.032), (0.091, 0.032)],
        segments=112,
    )
    scallops = _radial_box_ribs(
        x_center=0.0,
        length=0.040,
        radius=0.099,
        count=24,
        tangential_width=0.004,
        radial_height=0.0045,
        phase=math.pi / 24.0,
    )
    aperture_tab = _top_rib(x_center=-0.006, length=0.018, radius=0.100, width=0.030, height=0.008)
    return _merge_meshes([ring, scallops, aperture_tab])


def _build_inner_barrel_mesh() -> MeshGeometry:
    front_tube = _shell_x(
        [
            (0.057, -0.112),
            (0.061, -0.100),
            (0.061, 0.108),
            (0.070, 0.116),
            (0.070, 0.205),
            (0.058, 0.215),
            (0.067, 0.222),
            (0.067, 0.250),
            (0.058, 0.258),
        ],
        [
            (0.045, -0.114),
            (0.047, -0.100),
            (0.047, 0.108),
            (0.049, 0.116),
            (0.049, 0.250),
            (0.044, 0.258),
        ],
        segments=104,
    )
    retaining_notch = _top_rib(x_center=0.236, length=0.026, radius=0.067, width=0.010, height=0.004)
    return _merge_meshes([front_tube, retaining_notch])


def _build_focus_ring_mesh() -> MeshGeometry:
    ring = _shell_x(
        [
            (0.082, -0.048),
            (0.087, -0.038),
            (0.087, 0.038),
            (0.082, 0.048),
        ],
        [(0.074, -0.050), (0.074, 0.050)],
        segments=112,
    )
    fine_teeth = _radial_box_ribs(
        x_center=0.0,
        length=0.076,
        radius=0.086,
        count=44,
        tangential_width=0.0032,
        radial_height=0.005,
        phase=math.pi / 44.0,
    )
    witness = _top_rib(x_center=0.020, length=0.032, radius=0.087, width=0.008, height=0.005)
    return _merge_meshes([ring, fine_teeth, witness])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cine_zoom_lens")

    black_anodized = model.material("black_anodized", rgba=(0.010, 0.012, 0.014, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.060, 0.065, 0.070, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.42, 0.43, 0.40, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.015, 0.015, 0.013, 1.0))
    iris_blue = model.material("iris_blue", rgba=(0.07, 0.13, 0.20, 1.0))
    glass_coating = model.material("green_coated_glass", rgba=(0.12, 0.42, 0.36, 0.52))
    scale_white = model.material("engraved_white", rgba=(0.92, 0.90, 0.82, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_build_barrel_mesh(), "alloy_barrel"),
        material=dark_alloy,
        name="alloy_barrel",
    )
    barrel.visual(
        Box((0.006, 0.026, 0.0022)),
        origin=Origin(xyz=(-0.062, 0.0, 0.0895)),
        material=scale_white,
        name="zoom_index",
    )
    barrel.visual(
        Box((0.005, 0.020, 0.0022)),
        origin=Origin(xyz=(0.145, 0.0, 0.0868)),
        material=scale_white,
        name="iris_index",
    )
    barrel.visual(
        mesh_from_geometry(
            _radial_box_ribs(
                x_center=-0.055,
                length=0.014,
                radius=0.090,
                count=4,
                tangential_width=0.010,
                radial_height=0.010,
                phase=math.pi / 4.0,
            ),
            "zoom_bearings",
        ),
        material=satin_alloy,
        name="zoom_bearings",
    )
    barrel.visual(
        mesh_from_geometry(
            _radial_box_ribs(
                x_center=0.105,
                length=0.012,
                radius=0.087,
                count=4,
                tangential_width=0.009,
                radial_height=0.008,
            ),
            "iris_bearings",
        ),
        material=satin_alloy,
        name="iris_bearings",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_geometry(_build_zoom_ring_mesh(), "zoom_t_ring"),
        material=rubber_black,
        name="zoom_t_ring",
    )

    iris_ring = model.part("iris_ring")
    iris_ring.visual(
        mesh_from_geometry(_build_iris_ring_mesh(), "iris_aperture_ring"),
        material=iris_blue,
        name="iris_aperture_ring",
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        mesh_from_geometry(_build_inner_barrel_mesh(), "inner_front_barrel"),
        material=black_anodized,
        name="inner_front_barrel",
    )
    inner_barrel.visual(
        mesh_from_geometry(
            _radial_box_ribs(
                x_center=-0.070,
                length=0.018,
                radius=0.061,
                count=4,
                tangential_width=0.008,
                radial_height=0.012,
                phase=math.pi / 4.0,
            ),
            "slide_bearings",
        ),
        material=satin_alloy,
        name="slide_bearings",
    )
    inner_barrel.visual(
        mesh_from_geometry(
            _radial_box_ribs(
                x_center=0.160,
                length=0.014,
                radius=0.070,
                count=4,
                tangential_width=0.008,
                radial_height=0.006,
            ),
            "focus_bearings",
        ),
        material=satin_alloy,
        name="focus_bearings",
    )
    inner_barrel.visual(
        mesh_from_geometry(
            _x_lathe(
                LatheGeometry(
                    [
                        (0.000, 0.226),
                        (0.032, 0.229),
                        (0.0505, 0.236),
                        (0.0505, 0.244),
                        (0.032, 0.251),
                        (0.000, 0.254),
                    ],
                    segments=72,
                )
            ),
            "front_glass",
        ),
        material=glass_coating,
        name="front_glass",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(_build_focus_ring_mesh(), "focus_ring_grip"),
        material=satin_alloy,
        name="focus_ring_grip",
    )

    zoom_joint = model.articulation(
        "barrel_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=zoom_ring,
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "barrel_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.08, lower=0.0, upper=0.032),
        meta={"driven_by": zoom_joint.name, "coupling": "cam helix converts zoom-ring rotation to front-barrel travel"},
    )
    model.articulation(
        "barrel_to_iris_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=iris_ring,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.4, lower=0.0, upper=1.10),
    )
    model.articulation(
        "inner_barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.2, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    barrel = object_model.get_part("barrel")
    inner_barrel = object_model.get_part("inner_barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    iris_ring = object_model.get_part("iris_ring")

    zoom_joint = object_model.get_articulation("barrel_to_zoom_ring")
    inner_slide = object_model.get_articulation("barrel_to_inner_barrel")
    focus_joint = object_model.get_articulation("inner_barrel_to_focus_ring")
    iris_joint = object_model.get_articulation("barrel_to_iris_ring")

    ctx.allow_overlap(
        barrel,
        zoom_ring,
        elem_a="zoom_bearings",
        elem_b="zoom_t_ring",
        reason="Hidden bearing pads intentionally seat the rotating zoom T-ring on the outer sleeve.",
    )
    ctx.allow_overlap(
        barrel,
        iris_ring,
        elem_a="iris_bearings",
        elem_b="iris_aperture_ring",
        reason="Small hidden detent/bearing pads intentionally support the rotating iris aperture ring.",
    )
    ctx.allow_overlap(
        barrel,
        inner_barrel,
        elem_a="alloy_barrel",
        elem_b="slide_bearings",
        reason="The front barrel's slide pads intentionally ride inside the hollow outer sleeve.",
    )
    ctx.allow_overlap(
        focus_ring,
        inner_barrel,
        elem_a="focus_ring_grip",
        elem_b="focus_bearings",
        reason="Hidden bearing pads intentionally retain the focus ring on the moving front barrel.",
    )

    ctx.check(
        "zoom ring drives inner barrel",
        inner_slide.meta.get("driven_by") == "barrel_to_zoom_ring",
        details=f"meta={inner_slide.meta!r}",
    )
    ctx.check(
        "all control rings rotate about optical axis",
        tuple(zoom_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(focus_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(iris_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axes={zoom_joint.axis}, {focus_joint.axis}, {iris_joint.axis}",
    )
    ctx.expect_within(
        inner_barrel,
        barrel,
        axes="yz",
        inner_elem="inner_front_barrel",
        outer_elem="alloy_barrel",
        margin=0.002,
        name="inner barrel centered in outer sleeve",
    )
    ctx.expect_overlap(
        inner_barrel,
        barrel,
        axes="x",
        elem_a="inner_front_barrel",
        elem_b="alloy_barrel",
        min_overlap=0.12,
        name="collapsed inner barrel remains retained",
    )
    rest_pos = ctx.part_world_position(inner_barrel)
    with ctx.pose({zoom_joint: 1.25, inner_slide: 0.032}):
        extended_pos = ctx.part_world_position(inner_barrel)
        ctx.expect_overlap(
            inner_barrel,
            barrel,
            axes="x",
            elem_a="inner_front_barrel",
            elem_b="alloy_barrel",
            min_overlap=0.08,
            name="zoomed inner barrel remains inserted",
        )
    ctx.check(
        "zoom rotation extends front barrel",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.025,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.expect_overlap(
        zoom_ring,
        barrel,
        axes="x",
        elem_a="zoom_t_ring",
        elem_b="zoom_bearings",
        min_overlap=0.010,
        name="zoom ring seated on bearing pads",
    )
    ctx.expect_overlap(
        iris_ring,
        barrel,
        axes="x",
        elem_a="iris_aperture_ring",
        elem_b="iris_bearings",
        min_overlap=0.010,
        name="iris ring seated on bearing pads",
    )
    ctx.expect_overlap(
        inner_barrel,
        barrel,
        axes="x",
        elem_a="slide_bearings",
        elem_b="alloy_barrel",
        min_overlap=0.010,
        name="slide pads retained inside outer sleeve",
    )
    ctx.expect_overlap(
        focus_ring,
        inner_barrel,
        axes="x",
        elem_a="focus_ring_grip",
        elem_b="inner_front_barrel",
        min_overlap=0.06,
        name="focus ring surrounds front barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        inner_barrel,
        axes="x",
        elem_a="focus_ring_grip",
        elem_b="focus_bearings",
        min_overlap=0.010,
        name="focus ring seated on bearing pads",
    )
    ctx.expect_overlap(
        iris_ring,
        barrel,
        axes="x",
        elem_a="iris_aperture_ring",
        elem_b="alloy_barrel",
        min_overlap=0.04,
        name="iris ring sits on barrel mid section",
    )
    return ctx.report()


object_model = build_object_model()
